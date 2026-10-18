/**
  ******************************************************************************
  * @file    chassis_tune.c
  * @brief   PID 整定实验实现
  *
  * 实验 1：开环静态标定
  *   四轮下发同一占空比，逐档上升（50‰ ~ 1000‰）再逐档下降（负向同理），
  *   每档保持 400ms，取最后 100ms 的四轮平均转速。输出文本：
  *     MAP_BEGIN steps=26 hold_ms=400 avg_ms=100
  *     MAP,duty,rpm10_FL,rpm10_FR,rpm10_BL,rpm10_BR      （rpm10 = RPM×10 取整）
  *     MAP_END
  *   用途：判编码器方向、量启动死区、算占空比-转速斜率（KFF 的来源）。
  *
  * 实验 2：闭环阶跃响应
  *   四轮同目标轮速做 0 → +A → 0 → −A → 0 阶跃，1kHz 存 RAM，结束后整体回传。
  *   数据块格式（小端 int16 × 12 通道 × n 个样本）：
  *     STEP_BEGIN n=<样本数> ch=12 dt_us=1000
  *     <n × 24 字节>                     ch0~3=目标RPM ch4~7=实测RPM ch8~11=占空比‰
  *     STEP_END cksum=<累加校验>
  ******************************************************************************
  */
#include "chassis_tune.h"

#include "chassis_config.h"
#include "chassis_math.h"
#include "usart.h"

#include <stdio.h>
#include <string.h>

#if CHASSIS_TUNE_ENABLE

#define TUNE_UART               huart1
#define TUNE_START_DELAY_TICKS  1000u   /* 上电先静置 1s，等串口和电机都稳定 */
#define TUNE_CH_NUM             12      /* 每样本通道数：ref×4 + meas×4 + duty×4 */

typedef enum
{
    TUNE_PH_IDLE = 0,       /* 上电等待 */
    TUNE_PH_RUN,            /* 跑实验 */
    TUNE_PH_DUMP,           /* 数据回传 */
    TUNE_PH_DONE,
} tune_phase_t;

static tune_phase_t s_phase;
static uint32_t s_tick;     /* 实验内 1kHz 计数 */

static void tune_send(const void *buf, uint16_t len)
{
    (void)HAL_UART_Transmit(&TUNE_UART, (uint8_t *)buf, len, 2000u);
}

/** 大块数据分片发送（HAL 的 Size 参数是 uint16，整块 72KB 会被截断） */
static void tune_send_block(const void *buf, uint32_t len)
{
    const uint8_t *p = (const uint8_t *)buf;

    while (len > 0u)
    {
        uint32_t chunk = (len > 2048u) ? 2048u : len;
        (void)HAL_UART_Transmit(&TUNE_UART, (uint8_t *)p, (uint16_t)chunk, 5000u);
        p += chunk;
        len -= chunk;
    }
}

static void tune_print(const char *str)
{
    tune_send(str, (uint16_t)strlen(str));
}

#if ((CHASSIS_TUNE_EXPERIMENT == 2) || (CHASSIS_TUNE_EXPERIMENT == 4))
static int16_t to_i16(float v)
{
    return (int16_t)(v >= 0.0f ? v + 0.5f : v - 0.5f);
}
#endif

/* ========================== 实验 1：开环静态标定 ========================== */

#if (CHASSIS_TUNE_EXPERIMENT == 1)

#define MAP_HOLD_TICKS   400u
#define MAP_AVG_TICKS    100u

/* 正负各扫一遍：低占空比加密，用来找启动死区 */
static const float s_map_duty[] =
{
     50.0f, 100.0f, 150.0f, 200.0f, 250.0f, 300.0f,
    400.0f, 500.0f, 600.0f, 700.0f, 800.0f, 900.0f, 1000.0f,
    -50.0f, -100.0f, -150.0f, -200.0f, -250.0f, -300.0f,
    -400.0f, -500.0f, -600.0f, -700.0f, -800.0f, -900.0f, -1000.0f,
};
#define MAP_STEPS ((uint8_t)(sizeof(s_map_duty) / sizeof(s_map_duty[0])))

static float s_map_rpm[MAP_STEPS][CHASSIS_WHEEL_NUM];
static float s_map_acc[CHASSIS_WHEEL_NUM];
static uint16_t s_map_n;
static uint8_t s_map_cur;

static void map_finalize(uint8_t idx)
{
    for (int i = 0; i < CHASSIS_WHEEL_NUM; i++)
    {
        s_map_rpm[idx][i] = (s_map_n > 0u) ? (s_map_acc[i] / (float)s_map_n) : 0.0f;
    }
}

static void map_step(const chassis_state_t *st)
{
    uint32_t idx = s_tick / MAP_HOLD_TICKS;

    if (idx >= MAP_STEPS)
    {
        chassis_set_open_duty(0.0f);
        if (s_map_n > 0u)
        {
            map_finalize(s_map_cur);
        }
        s_phase = TUNE_PH_DUMP;
        return;
    }

    chassis_set_open_duty(s_map_duty[idx]);

    if ((uint8_t)idx != s_map_cur)
    {
        if (idx > 0u && s_map_n > 0u)
        {
            map_finalize(s_map_cur);
        }
        s_map_cur = (uint8_t)idx;
        for (int i = 0; i < CHASSIS_WHEEL_NUM; i++)
        {
            s_map_acc[i] = 0.0f;
        }
        s_map_n = 0;
    }

    if ((s_tick % MAP_HOLD_TICKS) >= (MAP_HOLD_TICKS - MAP_AVG_TICKS))
    {
        for (int i = 0; i < CHASSIS_WHEEL_NUM; i++)
        {
            s_map_acc[i] += st->wheel_rpm_meas[i];
        }
        s_map_n++;
    }
}

static void map_dump(void)
{
    char line[128];

    snprintf(line, sizeof(line), "MAP_BEGIN steps=%u hold_ms=400 avg_ms=100\r\n",
             (unsigned)MAP_STEPS);
    tune_print(line);

    for (uint8_t k = 0; k < MAP_STEPS; k++)
    {
        snprintf(line, sizeof(line), "MAP,%d,%d,%d,%d,%d\r\n",
                 (int)s_map_duty[k],
                 (int)(s_map_rpm[k][CHASSIS_WHEEL_FL] * 10.0f + (s_map_rpm[k][CHASSIS_WHEEL_FL] >= 0 ? 0.5f : -0.5f)),
                 (int)(s_map_rpm[k][CHASSIS_WHEEL_FR] * 10.0f + (s_map_rpm[k][CHASSIS_WHEEL_FR] >= 0 ? 0.5f : -0.5f)),
                 (int)(s_map_rpm[k][CHASSIS_WHEEL_BL] * 10.0f + (s_map_rpm[k][CHASSIS_WHEEL_BL] >= 0 ? 0.5f : -0.5f)),
                 (int)(s_map_rpm[k][CHASSIS_WHEEL_BR] * 10.0f + (s_map_rpm[k][CHASSIS_WHEEL_BR] >= 0 ? 0.5f : -0.5f)));
        tune_print(line);
    }
    tune_print("MAP_END\r\n");
}

#endif /* CHASSIS_TUNE_EXPERIMENT == 1 */

/* ========================== 实验 2：闭环阶跃响应 ========================== */

#if (CHASSIS_TUNE_EXPERIMENT == 2)

/* 一次烧录扫多组 PI 参数：每组跑一轮 0→+A→0→−A→0，跑完立刻回传。
 * kff 用 chassis_config.h 里的逐轮值，不动。 */
typedef struct { float kp, ki, kacc, rpm; } gain_set_t;

static const gain_set_t s_gain_sets[] =
{
    { 3.0f, 40.0f, 0.25f, 120.0f },
    { 3.0f, 40.0f, 0.20f, 120.0f },
    { 3.0f, 70.0f, 0.25f, 120.0f },
    { 3.0f, 40.0f, 0.25f,  25.0f },
};
#define GAIN_SET_NUM ((uint8_t)(sizeof(s_gain_sets) / sizeof(s_gain_sets[0])))

static uint8_t s_set_idx;

/* 参考轨迹：按"实际跑车"的方式做斜升（1500 RPM/s，与 CHASSIS_WHEEL_ACCEL_RPM_S 一致），
 * 而不是理想阶跃——阶跃会把 Kp 的冲击放大到不真实的地步。
 * 关键点 (t_ms, rpm)：静止 -> 斜升到 +A -> 保持 -> 斜升回 0 -> 静止 -> 负向同理 */
static const uint16_t s_kp_t[] = { 0u, 300u, 380u, 900u, 980u, 1200u, 1280u, 1800u, 1880u, 2000u };
static const float    s_kp_v[] = { 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, -1.0f, -1.0f, 0.0f, 0.0f };
#define KP_NUM      ((uint8_t)(sizeof(s_kp_t) / sizeof(s_kp_t[0])))
#define PLAN_TOTAL  2000u   /* 必须等于 s_kp_t[] 最后一项 */

static int16_t s_samp[PLAN_TOTAL][TUNE_CH_NUM];

static void plan_ref(uint32_t t, float *ref)
{
    for (uint8_t k = 1; k < KP_NUM; k++)
    {
        if (t <= s_kp_t[k])
        {
            float dt = (float)(s_kp_t[k] - s_kp_t[k - 1]);
            float a = (dt > 0.0f) ? ((float)(t - s_kp_t[k - 1]) / dt) : 1.0f;
            *ref = s_gain_sets[s_set_idx].rpm * (s_kp_v[k - 1] + a * (s_kp_v[k] - s_kp_v[k - 1]));
            return;
        }
    }
    *ref = 0.0f;
}

static void step_step(const chassis_state_t *st)
{
    float ref;
    float refs[CHASSIS_WHEEL_NUM];

    if (s_tick >= PLAN_TOTAL)
    {
        for (int i = 0; i < CHASSIS_WHEEL_NUM; i++)
        {
            refs[i] = 0.0f;
        }
        chassis_set_wheel_ref(refs);
        s_phase = TUNE_PH_DUMP;
        return;
    }

    plan_ref(s_tick, &ref);
    for (int i = 0; i < CHASSIS_WHEEL_NUM; i++)
    {
        refs[i] = ref;
    }
    chassis_set_wheel_ref(refs);

    for (int i = 0; i < CHASSIS_WHEEL_NUM; i++)
    {
        s_samp[s_tick][i] = to_i16(ref);
        s_samp[s_tick][4 + i] = to_i16(st->wheel_rpm_meas[i]);
        s_samp[s_tick][8 + i] = to_i16(st->wheel_duty[i]);
    }
}

static void step_dump(void)
{
    char line[64];
    uint32_t sum = 0;
    const uint8_t *p = (const uint8_t *)s_samp;
    uint32_t n = (uint32_t)PLAN_TOTAL * TUNE_CH_NUM * sizeof(int16_t);

    for (uint32_t k = 0; k < n; k++)
    {
        sum += p[k];
    }

    snprintf(line, sizeof(line), "SET idx=%u kp=%d ki=%d kacc=%d rpm=%d\r\n",
             (unsigned)s_set_idx,
             (int)(s_gain_sets[s_set_idx].kp * 100.0f + 0.5f),
             (int)(s_gain_sets[s_set_idx].ki * 100.0f + 0.5f),
             (int)(s_gain_sets[s_set_idx].kacc * 1000.0f + 0.5f),
             (int)s_gain_sets[s_set_idx].rpm);
    tune_print(line);

    snprintf(line, sizeof(line), "STEP_BEGIN n=%u ch=12 dt_us=1000\r\n",
             (unsigned)PLAN_TOTAL);
    tune_print(line);
    tune_send_block((const void *)s_samp, n);

    snprintf(line, sizeof(line), "\r\nSTEP_END cksum=%lu\r\n", (unsigned long)sum);
    tune_print(line);
}

#endif /* CHASSIS_TUNE_EXPERIMENT == 2 */

/* ===================== 实验 3：开环转向目检（人眼确认） ==================== */

#if (CHASSIS_TUNE_EXPERIMENT == 3)

static void dir_step(const chassis_state_t *st)
{
    uint32_t t = s_tick;
    float duty;

    if (t < 300u)
    {
        duty = 0.0f;
    }
    else if (t < 1800u)
    {
        duty = 300.0f;
        if (t == 300u)
        {
            tune_print("\r\nDIR +300 开始（应为四轮同时朝车头方向滚）\r\n");
        }
    }
    else if (t < 2600u)
    {
        duty = 0.0f;
    }
    else if (t < 4100u)
    {
        duty = -300.0f;
        if (t == 2600u)
        {
            tune_print("\r\nDIR -300 开始（应为四轮同时朝车尾方向滚）\r\n");
        }
    }
    else
    {
        duty = 0.0f;
        if (t > 5000u)
        {
            s_tick = 0;     /* 循环播放，方便反复观察 */
            tune_print("\r\nDIR 循环重来\r\n");
        }
    }
    chassis_set_open_duty(duty);
    (void)st;
}

#endif /* CHASSIS_TUNE_EXPERIMENT == 3 */

/* ===================== 实验 4：惯性滑行自证 ================================
 * 驱动 500‰ 转 1.5s -> 然后 STBY 拉低、PWM 全停（无任何开关噪声）
 * -> 观察编码器是否"靠惯性"平滑衰减到 0。
 * 有衰减 = 编码器测的是真转；瞬间归零 = 之前那些读数是驱动噪声。
 */
#if (CHASSIS_TUNE_EXPERIMENT == 4)

#define PLAN_TOTAL 3500u
static int16_t s_samp[PLAN_TOTAL][TUNE_CH_NUM];

static void coast_step(const chassis_state_t *st)
{
    uint32_t t = s_tick;
    float ref = 0.0f;

    if (t >= PLAN_TOTAL)
    {
        chassis_set_open_duty(0.0f);
        chassis_driver_enable(1);
        s_phase = TUNE_PH_DUMP;
        return;
    }

    if (t < 300u)
    {
        chassis_driver_enable(1);
        chassis_set_open_duty(0.0f);
    }
    else if (t < 1800u)
    {
        chassis_driver_enable(1);
        chassis_set_open_duty(500.0f);
        ref = 500.0f;
    }
    else
    {
        /* 驱动彻底断开：PWM 0 + 输出高阻，电机只剩轴承和齿轮阻力 */
        chassis_set_open_duty(0.0f);
        chassis_driver_enable(0);
    }

    for (int i = 0; i < CHASSIS_WHEEL_NUM; i++)
    {
        s_samp[t][i] = to_i16(ref);
        s_samp[t][4 + i] = to_i16(st->wheel_rpm_meas[i]);
        s_samp[t][8 + i] = to_i16(st->wheel_duty[i]);
    }
}

static void coast_dump(void)
{
    char line[64];
    uint32_t n = (uint32_t)PLAN_TOTAL * TUNE_CH_NUM * sizeof(int16_t);
    uint32_t sum = 0;
    const uint8_t *p = (const uint8_t *)s_samp;

    for (uint32_t k = 0; k < n; k++) { sum += p[k]; }
    snprintf(line, sizeof(line), "SET idx=0 coast kp=0 ki=0 kacc=0 rpm=500\r\n");
    tune_print(line);
    snprintf(line, sizeof(line), "STEP_BEGIN n=%u ch=12 dt_us=1000\r\n", (unsigned)PLAN_TOTAL);
    tune_print(line);
    tune_send_block((const void *)s_samp, n);
    snprintf(line, sizeof(line), "\r\nSTEP_END cksum=%lu\r\n", (unsigned long)sum);
    tune_print(line);
}

#endif /* CHASSIS_TUNE_EXPERIMENT == 4 */

/* =============================== 调度 ===================================== */

void chassis_tune_init(void)
{
    s_phase = TUNE_PH_IDLE;
    s_tick = 0;

#if ((CHASSIS_TUNE_EXPERIMENT == 1) || (CHASSIS_TUNE_EXPERIMENT == 3) || (CHASSIS_TUNE_EXPERIMENT == 4))
    chassis_set_mode(CHASSIS_MODE_OPEN_DUTY);
    chassis_set_open_duty(0.0f);
#if (CHASSIS_TUNE_EXPERIMENT == 1)
    s_map_n = 0;
    s_map_cur = 0;
    for (uint8_t k = 0; k < MAP_STEPS; k++)
    {
        for (int i = 0; i < CHASSIS_WHEEL_NUM; i++)
        {
            s_map_rpm[k][i] = 0.0f;
        }
    }
#endif
#elif (CHASSIS_TUNE_EXPERIMENT == 2)
    s_set_idx = 0;
    for (int i = 0; i < CHASSIS_WHEEL_NUM; i++)
    {
        chassis_set_wheel_gain(i, s_gain_sets[0].kp, s_gain_sets[0].ki,
                               s_gain_sets[0].kacc);
    }
    {
        float zero[CHASSIS_WHEEL_NUM] = { 0.0f, 0.0f, 0.0f, 0.0f };
        chassis_set_mode(CHASSIS_MODE_WHEEL_REF);
        chassis_set_wheel_ref(zero);
    }
#endif
}

void chassis_tune_on_tick(const chassis_state_t *st)
{
    if (s_phase == TUNE_PH_IDLE)
    {
        s_tick++;
        if (s_tick >= TUNE_START_DELAY_TICKS)
        {
            s_tick = 0;
            s_phase = TUNE_PH_RUN;
#if (CHASSIS_TUNE_EXPERIMENT == 1)
            tune_print("\r\nTUNE_START exp=1 (open-loop static map)\r\n");
#elif (CHASSIS_TUNE_EXPERIMENT == 3)
            tune_print("\r\nTUNE_START exp=3 (open-loop direction check)\r\n");
#elif (CHASSIS_TUNE_EXPERIMENT == 4)
            tune_print("\r\nTUNE_START exp=4 (coast-down proof)\r\n");
#elif (CHASSIS_TUNE_EXPERIMENT == 2)
            tune_print("\r\nTUNE_START exp=2 (closed-loop step sweep)\r\n");
#endif
        }
        return;
    }

    if (s_phase != TUNE_PH_RUN)
    {
        return;
    }

#if (CHASSIS_TUNE_EXPERIMENT == 1)
    map_step(st);
    s_tick++;
#elif (CHASSIS_TUNE_EXPERIMENT == 3)
    dir_step(st);
    s_tick++;
#elif (CHASSIS_TUNE_EXPERIMENT == 4)
    coast_step(st);
    s_tick++;
#elif (CHASSIS_TUNE_EXPERIMENT == 2)
    step_step(st);
    s_tick++;
#endif
}

void chassis_tune_poll(void)
{
    if (s_phase != TUNE_PH_DUMP)
    {
        return;
    }
    s_phase = TUNE_PH_DONE;     /* 先标记，避免重入 */

#if (CHASSIS_TUNE_EXPERIMENT == 1)
    map_dump();
    tune_print("TUNE_END\r\n");
#elif (CHASSIS_TUNE_EXPERIMENT == 3)
    tune_print("TUNE_END\r\n");
#elif (CHASSIS_TUNE_EXPERIMENT == 4)
    coast_dump();
    tune_print("TUNE_END\r\n");
#elif (CHASSIS_TUNE_EXPERIMENT == 2)
    step_dump();

    /* 还有下一组参数就接着跑 */
    s_set_idx++;
    if (s_set_idx < GAIN_SET_NUM)
    {
        for (int i = 0; i < CHASSIS_WHEEL_NUM; i++)
        {
            chassis_set_wheel_gain(i, s_gain_sets[s_set_idx].kp,
                                   s_gain_sets[s_set_idx].ki,
                                   s_gain_sets[s_set_idx].kacc);
        }
        s_tick = 0;
        s_phase = TUNE_PH_RUN;
    }
    else
    {
        tune_print("TUNE_END\r\n");
    }
#endif
}

int chassis_tune_busy(void)
{
    return (s_phase != TUNE_PH_DONE) && (s_phase != TUNE_PH_IDLE);
}

#endif /* CHASSIS_TUNE_ENABLE */
