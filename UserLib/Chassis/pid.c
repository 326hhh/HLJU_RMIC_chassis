/**
  ******************************************************************************
  * @file    pid.c
  * @brief   位置式 PI(D) 实现（带速度前馈与条件积分抗饱和）
  ******************************************************************************
  */
#include "pid.h"
#include "chassis_math.h"

void pid_init(pid_t *pid, float kp, float ki, float kd, float kff,
              float i_max, float out_max, float d_lpf_tau)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->kff = kff;
    pid->i_max = i_max;
    pid->out_max = out_max;
    pid->d_lpf_tau = d_lpf_tau;
    pid->err = 0.0f;
    pid->integral = 0.0f;
    pid->d_filt = 0.0f;
    pid->out = 0.0f;
    pid->last_meas = 0.0f;
    pid->first = 1;
}

void pid_reset(pid_t *pid)
{
    pid->err = 0.0f;
    pid->integral = 0.0f;
    pid->d_filt = 0.0f;
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
        /* 首次计算：微分状态从当前测量值起步，避免上电瞬间的微分冲击 */
        pid->last_meas = measurement;
        pid->d_filt = 0.0f;
        pid->first = 0;
    }

    /* 微分作用在测量值上（避免给定值突变带来的微分冲击），并做一阶低通：
     * 1kHz 下测速本身有 ±1 计数的量化噪声，不滤波的微分只会把噪声放大成占空比尖刺。 */
    float d_raw = (measurement - pid->last_meas) / dt;
    pid->last_meas = measurement;
    if (pid->d_lpf_tau > 0.0f)
    {
        float a = dt / (pid->d_lpf_tau + dt);
        pid->d_filt += a * (d_raw - pid->d_filt);
    }
    else
    {
        pid->d_filt = d_raw;
    }

    /* 速度前馈：ref 对应的稳态占空比。它承担主要的"给多少力"，
     * 闭环只负责补偿负载、电压、个体差异。 */
    float ff = pid->kff * setpoint;

    /* 候选积分（先限幅，防止积分项自己就饱和） */
    float i_cand = chassis_clampf(pid->integral + err * dt, -pid->i_max, pid->i_max);

    float out = pid->kp * err + pid->ki * i_cand - pid->kd * pid->d_filt + ff;
    float out_sat = chassis_clampf(out, -pid->out_max, pid->out_max);

    /* 条件积分抗饱和：输出已经顶到限幅、且误差还在往同方向推时，本次不吸收积分。
     * 否则占空比打满那段时间里积分会一路暴涨，退饱和时要花很久才吐出来。 */
    int sat_high = (out > pid->out_max);
    int sat_low = (out < -pid->out_max);
    if (!((sat_high && err > 0.0f) || (sat_low && err < 0.0f)))
    {
        pid->integral = i_cand;
    }

    pid->err = err;
    pid->out = out_sat;
    return out_sat;
}
