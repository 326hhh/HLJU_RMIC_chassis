/**
  ******************************************************************************
  * @file    chassis_debug.c
  * @brief   底盘调试输出：USART1 周期发送，支持两种协议
  *          - RawData 文本（人眼直读）
  *          - JustFloat 二进制（VOFA+ 画波形，小端 float + 帧尾 00 00 80 7F）
  *
  * JustFloat 通道表（CH_COUNT = 13）：
  *   ch0~3  : 目标轮速 ref (FL FR BL BR, RPM)
  *   ch4~7  : 实测轮速 meas (FL FR BL BR, RPM)
  *   ch8~11 : 输出占空比 duty (FL FR BL BR, 千分比)
  *   ch12   : 里程计 yaw (deg)
  ******************************************************************************
  */
#include "chassis_debug.h"

#include "chassis.h"
#include "chassis_config.h"
#include "chassis_math.h"
#include "usart.h"

#include <stdio.h>
#include <string.h>

#if CHASSIS_DEBUG_ENABLE

#define DEBUG_PERIOD_MS     250u
#define DEBUG_CH_NUM        13

static uint32_t s_last_send;

static void dbg_transmit(const uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)data, len, 10u);
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

    /* Cortex-M 为小端，直接按字节拷贝即为小端 float */
    memcpy(frame, ch, sizeof(ch));
    memcpy(frame + sizeof(ch), JUSTFLOAT_TAIL, sizeof(JUSTFLOAT_TAIL));
    dbg_transmit(frame, sizeof(frame));
}

#else

static void dbg_send_rawtext(const chassis_state_t *st)
{
    char line[128];

    /* 格式：t=xxxxx | FL rrrr/dddd FR rrrr/dddd BL rrrr/dddd BR rrrr/dddd | yaw=ddddeg
     * rrrr = 实测轮速(RPM，四舍五入), dddd = 输出占空比(千分比) */
    int n = snprintf(line, sizeof(line),
                     "t=%lu | FL %4d/%4d FR %4d/%4d BL %4d/%4d BR %4d/%4d | yaw=%ddeg\r\n",
                     (unsigned long)chassis_tick_ms(),
                     (int)(st->wheel_rpm_meas[CHASSIS_WHEEL_FL] + (st->wheel_rpm_meas[CHASSIS_WHEEL_FL] >= 0 ? 0.5f : -0.5f)),
                     (int)st->wheel_duty[CHASSIS_WHEEL_FL],
                     (int)(st->wheel_rpm_meas[CHASSIS_WHEEL_FR] + (st->wheel_rpm_meas[CHASSIS_WHEEL_FR] >= 0 ? 0.5f : -0.5f)),
                     (int)st->wheel_duty[CHASSIS_WHEEL_FR],
                     (int)(st->wheel_rpm_meas[CHASSIS_WHEEL_BL] + (st->wheel_rpm_meas[CHASSIS_WHEEL_BL] >= 0 ? 0.5f : -0.5f)),
                     (int)st->wheel_duty[CHASSIS_WHEEL_BL],
                     (int)(st->wheel_rpm_meas[CHASSIS_WHEEL_BR] + (st->wheel_rpm_meas[CHASSIS_WHEEL_BR] >= 0 ? 0.5f : -0.5f)),
                     (int)st->wheel_duty[CHASSIS_WHEEL_BR],
                     (int)(st->yaw * 180.0f / CHASSIS_PI));
    (void)n;
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

#endif /* CHASSIS_DEBUG_ENABLE */
