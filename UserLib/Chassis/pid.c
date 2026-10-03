/**
  ******************************************************************************
  * @file    pid.c
  * @brief   位置式 PID 实现
  ******************************************************************************
  */
#include "pid.h"
#include "chassis_math.h"

void pid_init(pid_t *pid, float kp, float ki, float kd, float i_max, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->i_max = i_max;
    pid->out_max = out_max;
    pid->err = 0.0f;
    pid->integral = 0.0f;
    pid->out = 0.0f;
    pid->last_meas = 0.0f;
    pid->first = 1;
}

float pid_calc(pid_t *pid, float setpoint, float measurement, float dt)
{
    if (dt <= 0.0f)
    {
        return pid->out;
    }

    float err = setpoint - measurement;

    if (pid->first)
    {
        pid->last_meas = measurement;
        pid->first = 0;
    }

    /* 微分作用在测量值上，避免给定值突变带来的微分冲击 */
    float d_meas = (measurement - pid->last_meas) / dt;
    pid->last_meas = measurement;

    pid->integral = chassis_clampf(pid->integral + err * dt, -pid->i_max, pid->i_max);

    float out = pid->kp * err + pid->ki * pid->integral - pid->kd * d_meas;
    out = chassis_clampf(out, -pid->out_max, pid->out_max);

    pid->err = err;
    pid->out = out;
    return out;
}
