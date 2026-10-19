/**
  ******************************************************************************
  * @file    uart_link.c
  * @brief   底盘控制链路实现：USART2 中断收帧 + 定长帧解析 + 看门狗 + 反馈帧
  *
  * 设计约束（每条都影响稳定性，改动前先想清楚）：
  *
  * 1. **接收不阻塞**：RX 用单字节中断灌进 256 B 环形缓冲，解析在主循环跑。
  *    绝不在中断里做帧解析（`chassis_poll()` 是 1 kHz，主循环随时可能被 IMU
  *    的 213 µs I2C 读卡住，只有环形缓冲扛得住）。
  * 2. **发送不阻塞**：反馈帧 32 B @ 115200 要 2.8 ms，用 `HAL_UART_Transmit_IT`
  *    排队，发不完就丢帧（`tx_drop`），**绝不做阻塞发送**——否则 1 kHz 控制环掉拍。
  * 3. **看门狗只清零不失能**：控制帧超时 → 目标速度清零（走斜坡减速），
  *    但 STBY 保持高。云台/底盘松轴受力会直接掉下来，比继续转更危险。
  * 只有显式 ESTOP 命令才拉低 STBY。
  * 4. **急停会退出小陀螺**：SPIN 模式下陀螺外环用 spin_rate 当参考，光调
  *    `chassis_stop()` 停不下来，必须先 `chassis_spin(0)` 退回 RATE 模式。
  * 5. **CRC 覆盖帧头**：帧头也进 CRC，避免把「长得像帧头的噪声」当成帧。
  ******************************************************************************
  */
#include "uart_link.h"

#include "uart_link_codec.h"
#include "uart_link_config.h"

#include "chassis.h"
#include "chassis_config.h"
#include "chassis_debug.h"
#include "chassis_gyro.h"
#include "chassis_math.h"
#include "imu.h"
#include "usart.h"

#include <string.h>

/* ================================ 内部状态 ================================= */

/** 公开统计 */
static uart_link_state_t s_stat;

/* ---- 接收：ISR 写 head，主循环读 tail（单生产者单消费者，无需临界区） ---- */
static uint8_t  s_rx_ring[ULINK_RX_RING_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static uint8_t  s_rx_byte;              /* HAL 单字节接收缓冲（ISR 用） */

/* ---- 发送：同一时刻只挂一帧，发完由 TxCplt 清标志 ---- */
static uint8_t  s_tx_frame[ULINK_FB_FRAME_LEN];
static volatile uint8_t s_tx_busy;
static uint16_t s_fb_seq;               /* 反馈帧自身序号 */

/* ---- 控制状态 ---- */
static uint32_t s_last_ctrl_ms;         /* 最近一次合法控制帧时刻 */
static uint32_t s_last_fb_ms;           /* 最近一次反馈帧时刻 */
static volatile uint8_t  s_ctrl_seen;   /* 是否收到过控制帧 */
static uint8_t  s_wdg_latched;          /* 看门狗已触发（下降沿只动作一次） */
static uint8_t  s_estop;                /* 急停锁存 */
static uint32_t s_wdg_trips;            /* 看门狗触发累计（进 status 用不到，给调试） */

/* ============================== 小工具函数 ================================= */

/** 弧度 -> 0.01°，带限幅（yaw 已归一化到 ±180°，i16 ±327.67° 够用） */
static int16_t ulink_rad_to_cdeg(float rad)
{
    float v = rad * (180.0f / CHASSIS_PI) * 100.0f;
    if (v > 32767.0f)
    {
        v = 32767.0f;
    }
    else if (v < -32767.0f)
    {
        v = -32767.0f;
    }
    return (int16_t)v;
}

static inline uint16_t ulink_ring_next(uint16_t idx)
{
    return (uint16_t)((idx + 1u) & (ULINK_RX_RING_SIZE - 1u));
}

/* ================================ 接收路径 ================================= */

/* 环形缓冲必须是 2 的幂，否则 & 取模是错的（编译期检查） */
typedef char ulink_ring_size_must_be_pow2[(ULINK_RX_RING_SIZE & (ULINK_RX_RING_SIZE - 1u)) == 0u ? 1 : -1];

static void ulink_rx_push(uint8_t b)
{
    uint16_t next = ulink_ring_next(s_rx_head);
    s_stat.rx_bytes++;
    if (next == s_rx_tail)
    {
        s_stat.rx_overflow++;       /* 主循环太慢，缓冲满了，丢最新字节 */
        return;
    }
    s_rx_ring[s_rx_head] = b;
    s_rx_head = next;
}

static inline uint16_t ulink_rx_avail(void)
{
    return (uint16_t)((s_rx_head - s_rx_tail) & (ULINK_RX_RING_SIZE - 1u));
}

static inline uint8_t ulink_rx_peek(uint16_t offset)
{
    return s_rx_ring[(uint16_t)((s_rx_tail + offset) & (ULINK_RX_RING_SIZE - 1u))];
}

static inline void ulink_rx_drop(uint16_t n)
{
    s_rx_tail = (uint16_t)((s_rx_tail + n) & (ULINK_RX_RING_SIZE - 1u));
}

/** 处理一帧已通过 CRC 的命令（帧内容见 uart_link.h 的表） */
static void ulink_handle_frame(const uint8_t *f)
{
    ulink_cmd_t c;
    bool        en;
    uint32_t    now = HAL_GetTick();

    if (!ulink_parse_cmd(f, &c))
    {
        return;
    }
    en = ((c.flags & ULINK_FLAG_EN) != 0u);

    s_stat.rx_frames++;
    s_stat.last_seq = c.seq;

    switch (c.cmd)
    {
        case ULINK_CMD_SPEED:
        {
            float vx = (float)c.vx_mm * ULINK_MM_S_TO_M_S;    /* mm/s -> m/s */
            float vy = (float)c.vy_mm * ULINK_MM_S_TO_M_S;
            float wz = (float)c.wz_cs * ULINK_WZ_UNIT_RAD_S;

            s_stat.cmd_speed++;
            s_last_ctrl_ms = now;
            s_ctrl_seen = 1u;
            s_wdg_latched = 0u;

            if (en)
            {
                /* 从急停/小陀螺状态恢复：驱动使能 + 退出 SPIN（SPIN 模式会无视用户 wz） */
                if (s_estop != 0u)
                {
                    chassis_driver_enable(1);
                    s_estop = 0u;
                }
                chassis_spin(0.0f);            /* 非 SPIN 状态下是空操作 */
                chassis_set_velocity(vx, vy, wz);
            }
            else
            {
                chassis_stop();
                chassis_gyro_reset_correction();
            }
            break;
        }

        case ULINK_CMD_ESTOP:
            /* 急停：先退出小陀螺，再清速度，最后断输出（STBY 拉低 = 电机自由） */
            chassis_spin(0.0f);
            chassis_stop();
            chassis_gyro_reset_correction();
            chassis_driver_enable(0);
            s_estop = 1u;
            s_last_ctrl_ms = now;
            s_ctrl_seen = 1u;
            s_wdg_latched = 0u;
            break;

        case ULINK_CMD_SPIN:
        {
            float rate = (float)c.wz_cs * ULINK_WZ_UNIT_RAD_S;

            s_last_ctrl_ms = now;
            s_ctrl_seen = 1u;
            s_wdg_latched = 0u;

            if (en)
            {
                if (s_estop != 0u)
                {
                    chassis_driver_enable(1);
                    s_estop = 0u;
                }
                chassis_spin(rate);
            }
            else
            {
                chassis_spin(0.0f);
                chassis_stop();
                chassis_gyro_reset_correction();
            }
            break;
        }

        case ULINK_CMD_CLEAR:
            /* 里程计 + 陀螺积分 yaw 一起归零（原地「重新定义原点」） */
            chassis_reset_odometry();
            chassis_gyro_reset();
            break;

        default:
            s_stat.cmd_unknown++;
            break;
    }
}

/** 从环形缓冲里切帧：丢噪声字节、校验 CRC、合法就立刻执行 */
static void ulink_parse_rx(void)
{
    while (ulink_rx_avail() >= ULINK_CMD_FRAME_LEN)
    {
        /* 找帧头：对不上就滚一个字节。噪声字节里出现 0xA5 的概率 1/256，
         * 后面还有 CRC 兜底，所以这里不做更复杂的同步 */
        if ((ulink_rx_peek(0u) != ULINK_CMD_HEAD0) ||
            (ulink_rx_peek(1u) != ULINK_CMD_HEAD1))
        {
            ulink_rx_drop(1u);
            s_stat.rx_sync_err++;
            continue;
        }

        uint8_t frame[ULINK_CMD_FRAME_LEN];
        for (uint16_t i = 0u; i < ULINK_CMD_FRAME_LEN; i++)
        {
            frame[i] = ulink_rx_peek(i);
        }

        if (ulink_crc8(frame, ULINK_CMD_FRAME_LEN - 1u) != frame[ULINK_CMD_FRAME_LEN - 1u])
        {
            ulink_rx_drop(1u);          /* 只丢一字节，帧头可能是新帧的开头 */
            s_stat.rx_crc_err++;
            continue;
        }

        ulink_rx_drop(ULINK_CMD_FRAME_LEN);
        ulink_handle_frame(frame);
    }
}

/* ================================ 发送路径 ================================= */

static void ulink_send(uint8_t *frame, uint16_t len)
{
    if (s_tx_busy != 0u)
    {
        s_stat.tx_drop++;               /* 上一帧还没发完，这一帧直接丢（反馈帧可丢） */
        return;
    }
    s_tx_busy = 1u;
    if (HAL_UART_Transmit_IT(&huart2, frame, len) != HAL_OK)
    {
        s_tx_busy = 0u;
        s_stat.tx_drop++;
        return;
    }
    s_stat.tx_frames++;
}

/** 组装并发送一帧反馈（字段表见 uart_link.h） */
static void ulink_send_feedback(uint32_t now)
{
    const chassis_state_t      *st  = chassis_state();
    const chassis_gyro_state_t *gy  = chassis_gyro_state();
    const imu_state_t          *imu = imu_state();
    ulink_fb_t fb;
    uint16_t   status = 0u;

    /* ---- status 位 ---- */
    if (imu->online)          { status |= ULINK_ST_IMU_ONLINE; }
    if (imu->bias_calibrated) { status |= ULINK_ST_IMU_CAL; }
    if (s_stat.link_ok)       { status |= ULINK_ST_LINK_OK; }
    if ((s_ctrl_seen != 0u) && ((now - s_last_ctrl_ms) > ULINK_CMD_TIMEOUT_MS))
    {
        status |= ULINK_ST_WATCHDOG;
    }
    if (s_estop)              { status |= ULINK_ST_ESTOP; }
    if (st->driver_on)        { status |= ULINK_ST_DRIVER_ON; }
    if (gy->mode == CHASSIS_GYRO_MODE_SPIN) { status |= ULINK_ST_SPIN; }
    if (gy->mode != CHASSIS_GYRO_MODE_OFF)  { status |= ULINK_ST_GYRO_ON; }

    /* ---- 内容（里程计 / 正解速度 / 陀螺 / 状态） ---- */
    fb.seq           = s_stat.last_seq;
    fb.x_mm          = (int32_t)(st->x * 1000.0f);
    fb.y_mm          = (int32_t)(st->y * 1000.0f);
    fb.yaw_cdeg      = ulink_rad_to_cdeg(st->yaw);
    fb.vx_mm         = (int16_t)(st->meas_vx * 1000.0f);
    fb.vy_mm         = (int16_t)(st->meas_vy * 1000.0f);
    fb.wz_cs         = (int16_t)(st->meas_wz * 100.0f);
    fb.gyro_yaw_cdeg = ulink_rad_to_cdeg(st->imu_yaw);
    fb.gyro_wz_cs    = (int16_t)(st->imu_wz * 100.0f);
    fb.status        = status;
    fb.cmd_cnt       = (uint16_t)(s_stat.rx_frames & 0xFFFFu);
    fb.fb_seq        = s_fb_seq++;
    for (int i = 0; i < 4; i++)
    {
        /* 四轮实测转速（0.01 RPM）：麦轮方向检查/排障就靠这四个数 */
        fb.wheel_rpm[i] = (int16_t)(st->wheel_rpm_meas[i] * 100.0f);
    }

    (void)ulink_build_fb(s_tx_frame, &fb);
    ulink_send(s_tx_frame, ULINK_FB_FRAME_LEN);
}

/* ================================ 看门狗 =================================== */

static void ulink_watchdog(uint32_t now)
{
    bool timed_out = (s_ctrl_seen != 0u) &&
                     ((now - s_last_ctrl_ms) > ULINK_CMD_TIMEOUT_MS);

    /* LINK_OK 的语义是「链路没死」（500 ms 内收到过），比看门狗宽松：
     * 断几帧会置 WATCHDOG 但 LINK_OK 还亮，上位机据此区分「抖动」和「断了」 */
    s_stat.link_ok = ((s_ctrl_seen != 0u) &&
                      ((now - s_last_ctrl_ms) < ULINK_CMD_LOST_MS)) ? 1u : 0u;

    if (timed_out)
    {
        if (s_wdg_latched == 0u)
        {
            s_wdg_latched = 1u;
            s_wdg_trips++;
            s_stat.wdg_trips = s_wdg_trips;
        }
        /* 只清零、不失能：STBY 保持高，电机被速度环锁在 0（受控滑停） */
        chassis_spin(0.0f);
        chassis_stop();
        chassis_gyro_reset_correction();
    }
}

/* ============================ HAL 回调（中断） ============================= */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        ulink_rx_push(s_rx_byte);
        (void)HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1u);   /* 重新挂接收 */
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        /* ORE/FE/NE：HAL 会自动结束本次接收，必须自己重挂，否则从此哑巴 */
        s_stat.rx_hw_err++;
        s_tx_busy = 0u;
        (void)HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1u);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        s_tx_busy = 0u;
    }
    chassis_debug_on_tx_cplt(huart);    /* USART1 调试发送完成 -> 清忙标志 */
}

/* ================================ 公开接口 ================================= */

void uart_link_init(void)
{
    memset(&s_stat, 0, sizeof(s_stat));
    s_stat.link_ok = 0u;

    s_rx_head = 0u;
    s_rx_tail = 0u;
    s_tx_busy = 0u;
    s_fb_seq = 0u;
    s_ctrl_seen = 0u;
    s_wdg_latched = 0u;
    s_estop = 0u;
    s_wdg_trips = 0u;

    s_last_ctrl_ms = HAL_GetTick();
    s_last_fb_ms = HAL_GetTick();

    /* 挂上第一帧接收。之后每一字节在 RxCplt 回调里自动重挂 */
    (void)HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1u);
}

void uart_link_poll(void)
{
    uint32_t now = HAL_GetTick();

    ulink_parse_rx();        /* 收帧 + 执行命令（内部不阻塞） */
    ulink_watchdog(now);     /* 控制帧超时保护 */

    if ((now - s_last_fb_ms) >= ULINK_FB_PERIOD_MS)
    {
        s_last_fb_ms = now;
        ulink_send_feedback(now);
    }
}

const uart_link_state_t *uart_link_state(void)
{
    return &s_stat;
}
