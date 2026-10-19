/**
  ******************************************************************************
  * @file    chassis_debug.c
  * @brief   底盘调试输出：USART1 周期发送，支持两种协议
  *          - RawData 文本（人眼直读）
  *          - JustFloat 二进制（VOFA+ 画波形，小端 float + 帧尾 00 00 80 7F）
  *
  * JustFloat 通道表（CH_COUNT = 17）：
  *   ch0~3  : 目标轮速 ref (FL FR BL BR, RPM)
  *   ch4~7  : 实测轮速 meas (FL FR BL BR, RPM)
  *   ch8~11 : 输出占空比 duty (FL FR BL BR, 千分比)
  *   ch12   : 里程计 yaw (deg)
 *   ch13   : 陀螺实测 wz (rad/s)        ch14 : 陀螺积分 yaw (deg)
 *   ch15   : 实际下发 wz_eff (rad/s)    ch16 : IMU 在线 (0/1)
  ******************************************************************************
  */
#include "chassis_debug.h"

#include "chassis.h"
#include "chassis_config.h"
#include "chassis_math.h"
#include "imu.h"
#include "usart.h"

#include <stdio.h>
#include <string.h>

#if CHASSIS_DEBUG_ENABLE

#define DEBUG_PERIOD_MS     250u
#define DEBUG_CH_NUM        17

static uint32_t s_last_send;

/* 非阻塞发送：缓冲 + 忙标志，发不完就丢这帧（调试输出可丢）。
 * 原因：HAL_UART_Transmit 阻塞发送 ~200 字节约 17ms，会拖慢 1kHz 控制环。 */
static uint8_t          s_dbg_tx_buf[256];
static volatile uint8_t s_dbg_tx_busy;

static void dbg_transmit(const uint8_t *data, uint16_t len)
{
    if (s_dbg_tx_busy != 0u)
    {
        return;
    }
    if (len > (uint16_t)sizeof(s_dbg_tx_buf))
    {
        len = (uint16_t)sizeof(s_dbg_tx_buf);
    }
    memcpy(s_dbg_tx_buf, data, len);
    if (HAL_UART_Transmit_IT(&huart1, s_dbg_tx_buf, len) == HAL_OK)
    {
        s_dbg_tx_busy = 1u;
    }
}

/** 在 HAL_UART_TxCpltCallback 里转发调用：USART1 发送完成 -> 清忙标志 */
void chassis_debug_on_tx_cplt(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        s_dbg_tx_busy = 0u;
    }
}

#if (CHASSIS_DEBUG_PROTOCOL == CHASSIS_DEBUG_PROTO_JUSTFLOAT)

/* JustFloat 帧尾：0x7F800000 小端 */
static const uint8_t JUSTFLOAT_TAIL[4] = { 0x00, 0x00, 0x80, 0x7F };

static void dbg_send_justfloat(const chassis_state_t *st)
{
    float ch[DEBUG_CH_NUM];
    uint8_t frame[DEBUG_CH_NUM * 4 + 4];

    for (int i = 0; i < CHASSIS_WHEEL_NUM; i++)
    {
        ch[i] = st->wheel_rpm_ref[i];               /* ch0~3  目标轮速 */
        ch[4 + i] = st->wheel_rpm_meas[i];          /* ch4~7  实测轮速 */
        ch[8 + i] = st->wheel_duty[i];              /* ch8~11 占空比   */
    }
    ch[12] = st->yaw * 180.0f / CHASSIS_PI;         /* ch12   yaw(deg) */
    ch[13] = st->imu_wz;                            /* ch13   陀螺 wz (rad/s) */
    ch[14] = st->imu_yaw * 180.0f / CHASSIS_PI;     /* ch14   陀螺积分 yaw(deg) */
    ch[15] = st->wz_eff;                            /* ch15   下发 wz (rad/s) */
    ch[16] = (float)st->imu_online;                 /* ch16   IMU 在线 */

    /* Cortex-M 为小端，直接按字节拷贝即为小端 float */
    memcpy(frame, ch, sizeof(ch));
    memcpy(frame + sizeof(ch), JUSTFLOAT_TAIL, sizeof(JUSTFLOAT_TAIL));
    dbg_transmit(frame, sizeof(frame));
}

#else

/** 定点格式化 ±d.ddd（nano printf 没开浮点，不能用 %f） */
static void dbg_fmt3(char *dst, size_t sz, float v)
{
    int neg = (v < 0.0f);
    float a = neg ? -v : v;
    int whole = (int)a;
    int frac = (int)((a - (float)whole) * 1000.0f + 0.5f);

    if (frac >= 1000)
    {
        frac = 0;
        whole += 1;
    }
    (void)snprintf(dst, sz, "%c%d.%03d", neg ? '-' : '+', whole, frac);
}

static void dbg_send_rawtext(const chassis_state_t *st)
{
    char line[300];
    char s_gz[16];
    char s_wz[16];

    dbg_fmt3(s_gz, sizeof(s_gz), st->imu_wz);
    dbg_fmt3(s_wz, sizeof(s_wz), st->wz_eff);

    /* 格式：t=xxxxx | FL rrrr/dddd ... | yaw=ddddeg | imu ok gz=+0.123 iyaw=+12deg wz=+0.456
     * rrrr = 实测轮速(RPM，四舍五入), dddd = 输出占空比(千分比) */
    (void)snprintf(line, sizeof(line),
                   "t=%lu | rpm FL %4d FR %4d BL %4d BR %4d | cnt %ld %ld %ld %ld | "
                   "yaw=%ddeg | imu %s gz=%s iyaw=%ddeg wz=%s | ac=%d,%d,%d us=%d hz=%d "
                   "e=%lu eps=%lu ad=%02X sw=%lu rec=%lu\r\n",
                   (unsigned long)chassis_tick_ms(),
                   (int)st->wheel_rpm_meas[CHASSIS_WHEEL_FL],
                   (int)st->wheel_rpm_meas[CHASSIS_WHEEL_FR],
                   (int)st->wheel_rpm_meas[CHASSIS_WHEEL_BL],
                   (int)st->wheel_rpm_meas[CHASSIS_WHEEL_BR],
                   (long)st->wheel_count[CHASSIS_WHEEL_FL],
                   (long)st->wheel_count[CHASSIS_WHEEL_FR],
                   (long)st->wheel_count[CHASSIS_WHEEL_BL],
                   (long)st->wheel_count[CHASSIS_WHEEL_BR],
                   (int)(st->yaw * 180.0f / CHASSIS_PI),
                   imu_state()->healthy ? "ok" : (imu_state()->online ? "STALE" : "--"),
                   s_gz,
                   (int)(st->imu_yaw * 180.0f / CHASSIS_PI),
                   s_wz,
                   (int)(imu_state()->accel_mps2[0] * 100.0f),
                   (int)(imu_state()->accel_mps2[1] * 100.0f),
                   (int)(imu_state()->accel_mps2[2] * 100.0f),
                   (int)imu_state()->read_us,
                   (int)imu_state()->sample_hz,
                   (unsigned long)imu_state()->error_count,
                   (unsigned long)imu_state()->err_per_s,
                   (unsigned)(imu_state()->i2c_addr >> 1),
                   (unsigned long)imu_state()->addr_switches,
                   (unsigned long)imu_state()->recover_count);
    dbg_transmit((const uint8_t *)line, (uint16_t)strlen(line));
}

#endif /* CHASSIS_DEBUG_PROTOCOL */

void chassis_debug_poll(void)
{
    uint32_t now = chassis_tick_ms();
    if (now - s_last_send < DEBUG_PERIOD_MS)
    {
        return;
    }
    s_last_send = now;

#if (CHASSIS_DEBUG_PROTOCOL == CHASSIS_DEBUG_PROTO_JUSTFLOAT)
    dbg_send_justfloat(chassis_state());
#else
    dbg_send_rawtext(chassis_state());
#endif
}

#else

void chassis_debug_poll(void)
{
    /* 编译期关闭 */
}

void chassis_debug_on_tx_cplt(UART_HandleTypeDef *huart)
{
    (void)huart;
}

#endif /* CHASSIS_DEBUG_ENABLE */
