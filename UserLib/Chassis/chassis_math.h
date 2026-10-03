/**
  ******************************************************************************
  * @file    chassis_math.h
  * @brief   底盘库通用小工具（纯头文件内联实现）
  ******************************************************************************
  */
#ifndef USERLIB_CHASSIS_CHASSIS_MATH_H
#define USERLIB_CHASSIS_CHASSIS_MATH_H

#define CHASSIS_PI          3.14159265f
#define CHASSIS_TWO_PI      6.28318531f
#define CHASSIS_RPM_TO_RAD_S    (CHASSIS_TWO_PI / 60.0f)   /* 1 RPM = 0.10472 rad/s */
#define CHASSIS_RAD_S_TO_RPM    (60.0f / CHASSIS_TWO_PI)   /* 1 rad/s = 9.5493 RPM  */

/** 数值限幅 */
static inline float chassis_clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/** 角度归一化到 [-PI, PI) */
static inline float chassis_wrap_pi(float a)
{
    while (a >= CHASSIS_PI)  a -= CHASSIS_TWO_PI;
    while (a < -CHASSIS_PI)  a += CHASSIS_TWO_PI;
    return a;
}

/** 斜坡限幅：cur 以不超过 rate(单位/s) 的斜率逼近 target */
static inline float chassis_rampf(float cur, float target, float rate, float dt)
{
    float step = rate * dt;
    if (target > cur + step) return cur + step;
    if (target < cur - step) return cur - step;
    return target;
}

#endif /* USERLIB_CHASSIS_CHASSIS_MATH_H */
