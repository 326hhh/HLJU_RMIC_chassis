/**
  ******************************************************************************
  * @file    can_link.c
  * @brief   底盘控制链路的 CAN 实现：把 12B 命令帧 / 40B 反馈帧搬到 CAN 上
  *
  *  设计要点（与 docs/CAN_CONTROL_PROTOCOL.md v0.3 一一对应）：
  *
  *   1) 协议零重写
  *      命令仍是 uart_link 的 12 字节帧（帧头 + CRC），反馈仍是 40 字节帧。
  *      本模块只做「CAN 8 字节 <-> 协议字节」的搬运：
  *        0x020 (8B) -> 拼出 12B 命令帧（flags 取自 0x021）-> ulink_apply_cmd12()
  *        40B 反馈   -> 按 8 字节切 5 帧 -> 0x110~0x114 依次发出
  *      因此命令落地、看门狗、CRC 全部复用，两条链路语义严格一致。
  *
  *   2) 反馈组完整性（§3.2 规则 4）
  *      5 帧不是原子的：可能被 ZDT 扩展帧插入，也可能中途丢帧。
  *      本端保证「一组 5 帧连续推入 TX 队列、组内不插别的帧」，
  *      接收端靠 CRC + 5ms 组超时 + 下个 0x110 冲掉未满组来兜底。
  *
  *   3) 发送策略：TX 队列连续发，不做 2ms 错开（§3.4 风险②）
  *      F407 只有 3 个发送邮箱，5 帧塞不进去，所以「能发就发、发不动排队」。
  *      主循环每轮都 pump 一次，实测组内延迟 ≤2ms（接收端超时设 5ms 够用）。
  *
  *   4) 急停只收不发
  *      F407 接收 0x000 立即急停；自身异常时**不广播**（当前无故障检测框架，
  *      且 bus-off 时根本发不出去）。详见 §3.5 边界分析。
  ******************************************************************************
  */

#include "can_link.h"

#include <string.h>

#include "can.h"                /* CubeMX 生成，提供 hcan1 */
#include "uart_link.h"          /* ulink_apply_cmd12 / ulink_build_fb_frame / 看门狗 */
#include "uart_link_codec.h"    /* CRC / 帧头常量 / 12B 帧布局 */
#include "can_test_config.h"    /* 仅用于 CANTEST_ENABLE 互斥检查 */
#include "chassis.h"            /* 0x021 EN=0 时直接停车（不走命令通道，不污染 cmd_cnt） */
#include "chassis_gyro.h"

#if CAN_LINK_ENABLE && CANTEST_ENABLE
#error "can_link 与 can_test 不能同时启用：两者都要初始化 CAN1，会互相打架"
#endif

/* ============================== 内部类型 ================================== */

typedef struct
{
    uint16_t id;
    uint8_t  dlc;
    uint8_t  data[CANLINK_FB_SPLIT_BYTES];
} canlink_tx_t;

/* ============================== 内部状态 ================================== */

static can_link_state_t s_stat;

static canlink_tx_t      s_txq[CANLINK_TX_QUEUE_SIZE];
static volatile uint16_t s_txq_head;    /* 写入位置 */
static volatile uint16_t s_txq_tail;    /* 读出位置 */

static uint16_t s_flags;                /* 最近一次 0x021 收到的 flags */
static uint32_t s_last_fb_ms;
static uint8_t  s_started;

/* ============================== TX 队列 =================================== */

static uint16_t can_link_txq_used(void)
{
    return (uint16_t)((s_txq_head - s_txq_tail) & 0xFFFFu);
}

static uint16_t can_link_txq_free(void)
{
    return (uint16_t)(CANLINK_TX_QUEUE_SIZE - can_link_txq_used());
}

static bool can_link_txq_push(uint16_t id, const uint8_t *data, uint8_t dlc)
{
    if (can_link_txq_free() == 0u)
    {
        s_stat.tx_queue_full++;
        return false;
    }
    canlink_tx_t *e = &s_txq[s_txq_head % CANLINK_TX_QUEUE_SIZE];

    e->id  = id;
    e->dlc = dlc;
    memset(e->data, 0, sizeof(e->data));
    if ((data != NULL) && (dlc > 0u))
    {
        memcpy(e->data, data, dlc);
    }
    s_txq_head++;
    return true;
}

/** 把队列里的帧尽量多地塞进发送邮箱（邮箱满就停，下次再来）。
 *  @note 只有 AddTxMessage 成功才推进 tail，失败则原样留着 —— 不丢帧。 */
static void can_link_pump_tx(void)
{
    while ((can_link_txq_used() > 0u) && (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0u))
    {
        const canlink_tx_t *e = &s_txq[s_txq_tail % CANLINK_TX_QUEUE_SIZE];
        CAN_TxHeaderTypeDef hdr;
        uint32_t            mailbox = 0u;

        memset(&hdr, 0, sizeof(hdr));
        hdr.StdId              = (uint16_t)(e->id & 0x7FFU);
        hdr.ExtId              = 0u;
        hdr.IDE                = CAN_ID_STD;
        hdr.RTR                = CAN_RTR_DATA;
        hdr.DLC                = e->dlc;
        hdr.TransmitGlobalTime = DISABLE;

        if (HAL_CAN_AddTxMessage(&hcan1, &hdr, (uint8_t *)e->data, &mailbox) != HAL_OK)
        {
            s_stat.tx_add_fail++;
            break;              /* 邮箱又被抢走了，下轮再试 */
        }

        s_txq_tail++;
        s_stat.tx_frames++;
    }
}

/* ============================== 收帧处理 ================================== */

/** 用 CAN 帧里的数据拼出 uart_link 的 12B 命令帧并下发。
 *  @param cmd      命令字
 *  @param seq      序号
 *  @param payload6 指向 6 字节负载（vx / vy / wz），可为 NULL 表示全 0
 */
static void can_link_apply(uint8_t cmd, uint8_t seq, const uint8_t *payload6)
{
    uint8_t f[ULINK_CMD_FRAME_LEN];

    f[0] = ULINK_CMD_HEAD0;
    f[1] = ULINK_CMD_HEAD1;
    f[2] = cmd;
    f[3] = seq;

    if (payload6 != NULL)
    {
        f[4] = payload6[0]; f[5] = payload6[1];     /* vx i16 LE */
        f[6] = payload6[2]; f[7] = payload6[3];     /* vy i16 LE */
        f[8] = payload6[4]; f[9] = payload6[5];     /* wz i16 LE */
    }
    else
    {
        memset(&f[4], 0, 6u);
    }

    f[10] = (uint8_t)(s_flags & 0xFFu);             /* flags 来自 0x021 */
    f[11] = ulink_crc8(f, ULINK_CMD_FRAME_LEN - 1u);

    (void)ulink_apply_cmd12(f);
}

static void can_link_handle_rx(void)
{
    while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0u)
    {
        CAN_RxHeaderTypeDef hdr;
        uint8_t             d[CANLINK_FB_SPLIT_BYTES];

        if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &hdr, d) != HAL_OK)
        {
            break;
        }

        /* 只看 11-bit 标准帧；ZDT 云台的 29-bit 扩展帧直接跳过 */
        if (hdr.IDE != CAN_ID_STD)
        {
            s_stat.rx_ignored++;
            continue;
        }

        switch (hdr.StdId)
        {
        case CANLINK_ID_ESTOP:
            /* 广播急停：DLC 无所谓，哪怕 0 字节也认。
             * 走 uart_link 的 ESTOP 分支：退小陀螺 -> 停车 -> 断 STBY -> 锁存 */
            can_link_apply(ULINK_CMD_ESTOP, 0u, NULL);
            s_stat.rx_frames++;
            break;

        case CANLINK_ID_CMD:
            if (hdr.DLC < 8u)
            {
                s_stat.rx_short++;      /* 速度命令必须满 8 字节 */
                break;
            }
            /* [cmd, seq, vx(2), vy(2), wz(2)] -> 12B 帧 */
            can_link_apply(d[0], d[1], &d[2]);
            s_stat.rx_frames++;
            break;

        case CANLINK_ID_FLAGS:
            if (hdr.DLC < 1u)
            {
                s_stat.rx_short++;
                break;
            }
            s_flags = d[0];
            s_stat.last_flags = s_flags;

            /* EN 被清掉时立刻停车，不用等下一个 0x020（安全起见）。
             * ⚠️ 这里**不走 ulink_apply_cmd12()**：0x021 是标志帧不是命令帧，
             *    走命令通道会让反馈里的 cmd_cnt 莫名 +1，破坏对方的丢包率统计。
             *    同时也故意不刷新 s_ctrl_seen —— EN=0 不是控制帧，看门狗该照常计时。 */
            if ((s_flags & ULINK_FLAG_EN) == 0u)
            {
                chassis_spin(0.0f);
                chassis_stop();
                chassis_gyro_reset_correction();
            }
            s_stat.rx_frames++;
            break;

        default:
            s_stat.rx_ignored++;
            break;
        }
    }
}

/* ============================ 发送：40B 反馈 =============================== */

/** 用 CAN_LINK 的位速率反读实际速率（不信配置信寄存器） */
static uint32_t can_link_read_bitrate(void)
{
    const uint32_t btr = hcan1.Instance->BTR;
    const uint32_t brp = (btr & CAN_BTR_BRP) + 1U;
    const uint32_t ts1 = ((btr & CAN_BTR_TS1) >> 16) + 1U;
    const uint32_t ts2 = ((btr & CAN_BTR_TS2) >> 20) + 1U;

    return CAN_LINK_APB1_HZ / brp / (1U + ts1 + ts2);
}

/** 每 CANLINK_FB_PERIOD_MS 组一帧 40B 反馈，切成 5 个 CAN 帧推入队列。
 *  @note 整组要么全进队列、要么全丢 —— 不发送残缺组，免得接收端拼出半帧。 */
static void can_link_send_feedback(uint32_t now)
{
    uint8_t fb[ULINK_FB_FRAME_LEN];

    if (ulink_build_fb_frame(fb, now) != ULINK_FB_FRAME_LEN)
    {
        return;
    }

    if (can_link_txq_free() < CANLINK_FB_SPLIT)
    {
        s_stat.tx_queue_full++;     /* 队列装不下一整组，整组丢弃 */
        return;
    }

    for (uint8_t i = 0u; i < CANLINK_FB_SPLIT; i++)
    {
        (void)can_link_txq_push((uint16_t)(CANLINK_ID_FB_BASE + i),
                                &fb[i * CANLINK_FB_SPLIT_BYTES],
                                CANLINK_FB_SPLIT_BYTES);
    }
}

/* ============================ CAN 错误监测 ================================ */

/** 读 CAN_ESR 的 TEC/REC，超阈值就把 CAN_ERR 位注入到反馈 status 里。
 *  为什么用 96：CAN 规范里 TEC/REC > 96 是 error-passive 的预警线，
 *  到 127 就真的降级了，提前一档告警才有意义。 */
static void can_link_update_err_flag(void)
{
    const uint32_t esr = hcan1.Instance->ESR;
    const uint32_t tec = (esr >> 16) & 0xFFU;
    const uint32_t rec = (esr >> 24) & 0xFFU;

    s_stat.can_err = ((tec > CANLINK_ERR_COUNT_THRESHOLD) ||
                      (rec > CANLINK_ERR_COUNT_THRESHOLD)) ? 1u : 0u;

    ulink_set_extra_status((s_stat.can_err != 0u) ? ULINK_ST_CAN_ERR : 0u);
}

/* ================================ 初始化 ================================== */

static void can_link_force_timing(void)
{
    hcan1.Init.Prescaler            = CAN_LINK_PRESCALER;
    hcan1.Init.Mode                 = CAN_MODE_NORMAL;   /* 真总线，不是 loopback */
    hcan1.Init.SyncJumpWidth        = CAN_LINK_SJW;
    hcan1.Init.TimeSeg1             = CAN_LINK_BS1;
    hcan1.Init.TimeSeg2             = CAN_LINK_BS2;
    hcan1.Init.TimeTriggeredMode    = DISABLE;
    hcan1.Init.AutoBusOff           = ENABLE;    /* §6：ABOM 硬件自动恢复，不锁死 */
    hcan1.Init.AutoWakeUp           = DISABLE;
    hcan1.Init.AutoRetransmission   = ENABLE;    /* 有真节点应答，正常重传 */
    hcan1.Init.ReceiveFifoLocked    = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;

    if (HAL_CAN_Init(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }
}

static void can_link_config_filter(void)
{
    CAN_FilterTypeDef f;

    memset(&f, 0, sizeof(f));
    f.FilterBank           = 0u;
    f.FilterMode           = CAN_FILTERMODE_IDMASK;
    f.FilterScale          = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh         = 0x0000u;
    f.FilterIdLow          = 0x0000u;
    f.FilterMaskIdHigh     = 0x0000u;    /* 掩码全 0 = 全通过（扩展帧也收，软件里挑） */
    f.FilterMaskIdLow      = 0x0000u;
    f.FilterFIFOAssignment = CAN_RX_FIFO0;
    f.FilterActivation     = ENABLE;
    f.SlaveStartFilterBank = 14u;        /* CAN2 的起始组（本模块不用 CAN2） */

    if (HAL_CAN_ConfigFilter(&hcan1, &f) != HAL_OK)
    {
        Error_Handler();
    }
}

void can_link_init(void)
{
    memset(&s_stat, 0, sizeof(s_stat));

    s_txq_head  = 0u;
    s_txq_tail  = 0u;
    s_flags     = 0u;
    s_started   = 0u;
    s_last_fb_ms = HAL_GetTick();

    /* 顺序硬要求：强制位时序 -> 配滤波器 -> Start（反过来 ConfigFilter 会报错） */
    can_link_force_timing();
    can_link_config_filter();

    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }

    s_started = 1u;
    s_stat.started = 1u;
    s_stat.bitrate_hz = can_link_read_bitrate();   /* 反读实际生效的速率，供调试器核对 */
}

/* ================================ 主循环 ================================== */

void can_link_poll(void)
{
    const uint32_t now = HAL_GetTick();

    if (s_started == 0u)
    {
        return;
    }

    can_link_handle_rx();       /* 1. 收：0x000 / 0x020 / 0x021 -> 底盘 */
    ulink_tick_watchdog(now);   /* 2. 看门狗（与串口共用同一套超时状态） */
    can_link_update_err_flag(); /* 3. CAN 错误计数 -> status 的 CAN_ERR 位 */

    if ((uint32_t)(now - s_last_fb_ms) >= CANLINK_FB_PERIOD_MS)
    {
        s_last_fb_ms = now;
        can_link_send_feedback(now);    /* 4. 100 Hz：组一组 40B 反馈推入队列 */
    }

    can_link_pump_tx();         /* 5. 能发就发（越勤组内延迟越低） */
}

const can_link_state_t *can_link_state(void)
{
    return &s_stat;
}
