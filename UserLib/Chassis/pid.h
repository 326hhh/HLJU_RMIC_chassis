/**
  ******************************************************************************
  * @file    pid.h
  * @brief   位置式 PI(D) + 速度前馈 + 条件积分抗饱和，微分作用在测量值上
  *
  * 控制律（每个控制周期）：
  *   ff  = kff * ref                      速度前馈：给出"维持该转速需要的占空比"
  *   e   = ref - meas
  *   I  += e * dt                         （条件积分：饱和时不继续积）
  *   d   = LPF( d(meas)/dt )              一阶低通后的测量微分
  *   out = kp*e + ki*I - kd*d + ff
  *   out = clamp(out, -out_max, out_max)
  ******************************************************************************
  */
#ifndef USERLIB_CHASSIS_PID_H
#define USERLIB_CHASSIS_PID_H

#include <stdint.h>

typedef struct
{
    float kp, ki, kd;       /* 增益 */
    float kff;              /* 速度前馈（占空比 / RPM） */
    float i_max, out_max;   /* 积分限幅 / 输出限幅 */
    float d_lpf_tau;        /* 微分一阶低通时间常数 (s)，<=0 表示不滤波 */
    float err;              /* 最近一次误差（诊断用） */
    float integral;         /* 积分项 */
    float d_filt;           /* 低通后的测量微分 (RPM/s) */
    float out;              /* 最近一次输出 */
    float last_meas;        /* 上一次测量值（微分用） */
    uint8_t first;          /* 首次计算标志 */
} pid_t;

void pid_init(pid_t *pid, float kp, float ki, float kd, float kff,
              float i_max, float out_max, float d_lpf_tau);

/** 清零积分与微分状态（切换工作模式/重新起调时调用，避免残留量带来的冲击） */
void pid_reset(pid_t *pid);

/**
 * @brief PID 计算
 * @param dt 控制周期 (s)，<=0 时直接返回上次输出
 */
float pid_calc(pid_t *pid, float setpoint, float measurement, float dt);

#endif /* USERLIB_CHASSIS_PID_H */
