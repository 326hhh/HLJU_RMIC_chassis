/**
  ******************************************************************************
  * @file    pid.h
  * @brief   位置式 PID，带积分限幅与输出限幅，微分作用在测量值上
  ******************************************************************************
  */
#ifndef USERLIB_CHASSIS_PID_H
#define USERLIB_CHASSIS_PID_H

#include <stdint.h>

typedef struct
{
    float kp, ki, kd;       /* 增益 */
    float i_max, out_max;   /* 积分限幅 / 输出限幅 */
    float err;              /* 最近一次误差（诊断用） */
    float integral;         /* 积分项 */
    float out;              /* 最近一次输出 */
    float last_meas;        /* 上一次测量值（微分用） */
    uint8_t first;          /* 首次计算标志 */
} pid_t;

void pid_init(pid_t *pid, float kp, float ki, float kd, float i_max, float out_max);

/**
 * @brief PID 计算
 * @param dt 控制周期 (s)，<=0 时直接返回上次输出
 */
float pid_calc(pid_t *pid, float setpoint, float measurement, float dt);

#endif /* USERLIB_CHASSIS_PID_H */
