/**
  ******************************************************************************
  * @file    mecanum.h
  * @brief   麦克纳姆轮运动学：正解 / 逆解
  *
  * 坐标系（RM 约定）：+x 前进，+y 左移，+wz 逆时针为正（俯视）
  * 轮序：FL(0) FR(1) BL(2) BR(3)，轮速单位 RPM，正=沿 +x 向前滚动
  ******************************************************************************
  */
#ifndef USERLIB_CHASSIS_MECANUM_H
#define USERLIB_CHASSIS_MECANUM_H

typedef enum
{
    CHASSIS_WHEEL_FL = 0,
    CHASSIS_WHEEL_FR = 1,
    CHASSIS_WHEEL_BL = 2,
    CHASSIS_WHEEL_BR = 3,
    CHASSIS_WHEEL_NUM = 4
} chassis_wheel_t;

/**
 * @brief 逆解：车体速度 -> 四轮目标转速
 * @param vx,vy 线速度 (m/s)，wz 角速度 (rad/s, 逆时针为正)
 * @param wheel_rpm 输出四轮转速 (RPM)
 */
void mecanum_inverse(float vx, float vy, float wz, float wheel_rpm[CHASSIS_WHEEL_NUM]);

/**
 * @brief 正解：四轮转速 -> 车体速度（里程计用）
 * @param wheel_rpm 四轮实测转速 (RPM)
 * @param vx,vy,wz 输出车体速度
 */
void mecanum_forward(const float wheel_rpm[CHASSIS_WHEEL_NUM],
                     float *vx, float *vy, float *wz);

#endif /* USERLIB_CHASSIS_MECANUM_H */
