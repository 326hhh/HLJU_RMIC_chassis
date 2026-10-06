/**
  ******************************************************************************
  * @file    chassis.c
  * @brief   麦克纳姆轮底盘主模块实现
  ******************************************************************************
  */
#include "chassis.h"
#include "chassis_config.h"
#include "chassis_math.h"
#include "encoder.h"
#include "main.h"
#include "motor.h"
#include "pid.h"
#include "tim.h"

#include <math.h>
#include <string.h>

/* ============================= 硬件绑定表 ===================================
 * 轮序固定：FL(0)=左前 FR(1)=右前 BL(2)=左后 BR(3)=右后
 * 接线（2025-10-06 重新布线版，见 README.md）：
 *   FL(左前): PWM=LF_PWMA=PA1(TIM5_CH2), 方向=L_AIN1(PC4)/L_AIN2(PC5), 编码器=TIM3(PA6/PA7)
 *   FR(右前): PWM=RF_PWMA=PA3(TIM5_CH4), 方向=R_AIN1(PE8)/R_AIN2(PE10), 编码器=TIM1(PE9/PE11)
 *   BL(左后): PWM=LB_PWMB=PA2(TIM5_CH3), 方向=L_BIN1(PB0)/L_BIN2(PB1), 编码器=TIM4(PD12/PD13)
 *   BR(右后): PWM=RB_PWMB=PE5(TIM9_CH1), 方向=R_BIN1(PE12)/R_BIN2(PE13), 编码器=TIM8(PC6/PC7)
 * fwd_in1/fwd_in2 = 该轮"正转"时 TB6612 IN1/IN2 电平。
 * 注意：方向电平沿用了旧线束的标定惯例（左轮 fwd=(0,1)、右轮 fwd=(1,0)），
 * 新线束/新驱动板首次上电必须按 docs/CHASSIS_LIBRARY.md §10 重新标定：
 * 某轮实际转向反了 -> 把该轮两个电平对调即可。
 */
typedef struct
{
    TIM_HandleTypeDef *pwm_htim;
    uint32_t pwm_ch;
    GPIO_TypeDef *in1_port;
    uint16_t in1_pin;
    GPIO_TypeDef *in2_port;
    uint16_t in2_pin;
    GPIO_PinState fwd_in1;
    GPIO_PinState fwd_in2;
    TIM_HandleTypeDef *enc_htim;
} wheel_bind_t;

static const wheel_bind_t s_bind[CHASSIS_WHEEL_NUM] =
{
    /* FL */ { &htim5, TIM_CHANNEL_2, GPIOC, L_AIN1_Pin, GPIOC, L_AIN2_Pin,
               GPIO_PIN_RESET, GPIO_PIN_SET, &htim3 },
    /* FR */ { &htim5, TIM_CHANNEL_4, GPIOE, R_AIN1_Pin, GPIOE, R_AIN2_Pin,
               GPIO_PIN_SET, GPIO_PIN_RESET, &htim1 },
    /* BL */ { &htim5, TIM_CHANNEL_3, GPIOB, L_BIN1_Pin, GPIOB, L_BIN2_Pin,
               GPIO_PIN_RESET, GPIO_PIN_SET, &htim4 },
    /* BR */ { &htim9, TIM_CHANNEL_1, GPIOE, R_BIN1_Pin, GPIOE, R_BIN2_Pin,
               GPIO_PIN_SET, GPIO_PIN_RESET, &htim8 },
};

/* ============================ 模块实例 ===================================== */
static motor_t s_motor[CHASSIS_WHEEL_NUM];
static encoder_t s_enc[CHASSIS_WHEEL_NUM];
static pid_t s_pid[CHASSIS_WHEEL_NUM];
static float s_ref_rpm[CHASSIS_WHEEL_NUM];
static chassis_state_t s_state;
static uint32_t s_last_tick;
static bool s_inited;

/* 每轮编码器测量方向归一系数（见 chassis_config.h），只作用于测量值。
 * 电机转向只由 s_bind[] 的 fwd_in1/fwd_in2 决定，两者互不干扰。 */
static const float s_enc_dir[CHASSIS_WHEEL_NUM] =
{
    (float)CHASSIS_ENC_DIR_FL, (float)CHASSIS_ENC_DIR_FR,
    (float)CHASSIS_ENC_DIR_BL, (float)CHASSIS_ENC_DIR_BR,
};

/* 每轮独立 PID 增益（轮序 FL FR BL BR，数值见 chassis_config.h） */
typedef struct
{
    float kp, ki, kd, i_max;
    float kff;      /* 速度前馈：占空比 = PID 输出 + kff * 目标轮速 */
} pid_gain_t;

static const pid_gain_t s_pid_gain[CHASSIS_WHEEL_NUM] =
{
    { CHASSIS_PID_KP_FL, CHASSIS_PID_KI_FL, CHASSIS_PID_KD_FL, CHASSIS_PID_I_MAX_FL, CHASSIS_PID_KFF_FL },
    { CHASSIS_PID_KP_FR, CHASSIS_PID_KI_FR, CHASSIS_PID_KD_FR, CHASSIS_PID_I_MAX_FR, CHASSIS_PID_KFF_FR },
    { CHASSIS_PID_KP_BL, CHASSIS_PID_KI_BL, CHASSIS_PID_KD_BL, CHASSIS_PID_I_MAX_BL, CHASSIS_PID_KFF_BL },
    { CHASSIS_PID_KP_BR, CHASSIS_PID_KI_BR, CHASSIS_PID_KD_BR, CHASSIS_PID_I_MAX_BR, CHASSIS_PID_KFF_BR },
};

/* ============================= 实现 ======================================== */

static void chassis_update(float dt);

uint32_t chassis_tick_ms(void)
{
    return HAL_GetTick();
}

void chassis_init(void)
{
    /* TB6612 使能 */
    HAL_GPIO_WritePin(L_STBY_GPIO_Port, L_STBY_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(R_STBY_GPIO_Port, R_STBY_Pin, GPIO_PIN_SET);

    for (int i = 0; i < CHASSIS_WHEEL_NUM; i++)
    {
        const wheel_bind_t *b = &s_bind[i];
        motor_init(&s_motor[i], b->pwm_htim, b->pwm_ch,
                   b->in1_port, b->in1_pin, b->in2_port, b->in2_pin,
                   b->fwd_in1, b->fwd_in2);
        encoder_start(&s_enc[i], b->enc_htim);
        pid_init(&s_pid[i], s_pid_gain[i].kp, s_pid_gain[i].ki, s_pid_gain[i].kd,
                 s_pid_gain[i].i_max, CHASSIS_PID_OUT_MAX);
        s_ref_rpm[i] = 0.0f;
    }

    memset(&s_state, 0, sizeof(s_state));
    s_last_tick = HAL_GetTick();
    s_inited = true;
}

void chassis_poll(void)
{
    if (!s_inited)
    {
        return;
    }

    uint32_t now = HAL_GetTick();
    if (now == s_last_tick)
    {
        return;                 /* 未到 1ms 节拍 */
    }

    uint32_t dt_ms = now - s_last_tick;
    s_last_tick = now;
    if (dt_ms > 5u)
    {
        dt_ms = 5u;             /* 调试断点等长停顿后限幅，防止 dt 爆炸 */
    }
    chassis_update(dt_ms * 0.001f);
}

void chassis_set_velocity(float vx, float vy, float wz)
{
    s_state.cmd_vx = chassis_clampf(vx, -CHASSIS_MAX_VX_M_S, CHASSIS_MAX_VX_M_S);
    s_state.cmd_vy = chassis_clampf(vy, -CHASSIS_MAX_VY_M_S, CHASSIS_MAX_VY_M_S);
    s_state.cmd_wz = chassis_clampf(wz, -CHASSIS_MAX_WZ_RAD_S, CHASSIS_MAX_WZ_RAD_S);
}

void chassis_stop(void)
{
    chassis_set_velocity(0.0f, 0.0f, 0.0f);
}

const chassis_state_t *chassis_state(void)
{
    return &s_state;
}

/* ============================ 1kHz 控制循环 ================================ */

static void chassis_update(float dt)
{
    float kin_rpm[CHASSIS_WHEEL_NUM];

    /* 1. 运动学逆解：车体指令 -> 轮速指令 */
    mecanum_inverse(s_state.cmd_vx, s_state.cmd_vy, s_state.cmd_wz, kin_rpm);

    for (int i = 0; i < CHASSIS_WHEEL_NUM; i++)
    {
        /* 2. 测量：编码器采样 + 滑动窗口测速 + 方向归一 */
        encoder_update(&s_enc[i]);
        s_state.wheel_count[i] = encoder_total(&s_enc[i]);
        s_state.wheel_rpm_meas[i] = s_enc_dir[i] * encoder_speed_rpm(&s_enc[i]);

        /* 3. 轮速限幅 + 斜坡 */
        kin_rpm[i] = chassis_clampf(kin_rpm[i], -CHASSIS_MAX_WHEEL_RPM,
                                    CHASSIS_MAX_WHEEL_RPM);
        s_ref_rpm[i] = chassis_rampf(s_ref_rpm[i], kin_rpm[i],
                                     CHASSIS_WHEEL_ACCEL_RPM_S, dt);
        s_state.wheel_rpm_ref[i] = s_ref_rpm[i];

        /* 4. 速度环：PID + 速度前馈 -> 占空比 -> 电机
         *    （正输出 = fwd 电平 = 向前滚；前馈按斜坡后的目标轮速给） */
        float out = pid_calc(&s_pid[i], s_ref_rpm[i],
                             s_state.wheel_rpm_meas[i], dt)
                    + s_pid_gain[i].kff * s_ref_rpm[i];
        out = chassis_clampf(out, -CHASSIS_PID_OUT_MAX, CHASSIS_PID_OUT_MAX);
        s_state.wheel_duty[i] = out;
        motor_set_duty(&s_motor[i], out);
    }

    /* 5. 运动学正解 + 里程计积分 */
    mecanum_forward(s_state.wheel_rpm_meas,
                    &s_state.meas_vx, &s_state.meas_vy, &s_state.meas_wz);

    float c = cosf(s_state.yaw);
    float s = sinf(s_state.yaw);
    s_state.x += (s_state.meas_vx * c - s_state.meas_vy * s) * dt;
    s_state.y += (s_state.meas_vx * s + s_state.meas_vy * c) * dt;
    s_state.yaw = chassis_wrap_pi(s_state.yaw + s_state.meas_wz * dt);
}
