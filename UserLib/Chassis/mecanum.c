/**
  ******************************************************************************
  * @file    mecanum.c
  * @brief   麦克纳姆轮运动学实现
  *
  * 逆解公式（推导见 docs/CHASSIS_LIBRARY.md 附录）：
  *   omega_i = ( vx + S*ky_i*vy + S*kw_i*wz*L ) / r
  *   其中 L = lx + ly（半轴距+半轮距），S = CHASSIS_ROLLER_SIGN
  *   ky = {-1,+1,+1,-1},  kw = {-1,+1,-1,+1}  （FL FR BL BR 顺序）
  ******************************************************************************
  */
#include "mecanum.h"
#include "chassis_config.h"
#include "chassis_math.h"

/* 每轮 vy / wz 项的符号系数（FL FR BL BR）
 * ⚠ 两行必须「同源」：vy 与 wz 的符号都来自同一套辊筒方向 σ（见 docs/CHASSIS_LIBRARY.md 附录 A）。
 *   单独翻某一行来"修方向"会重新变成不自洽的假表——2026-11 连续踩了两次：
 *     ① wz 写成前后对转 {-1,-1,+1,+1} → 纯 wz 指令变「往前窜 + 画弧」；
 *     ② vy 写成 σ 的负号形式 {+1,-1,-1,+1} → A/D 平移方向相反；
 *   实车（标准安装 σ = {-1,+1,+1,-1}）对应：ky = {-1,+1,+1,-1}、kw = {-1,+1,-1,+1}。 */
static const float K_VY[CHASSIS_WHEEL_NUM] = { -1.0f, +1.0f, +1.0f, -1.0f };
static const float K_WZ[CHASSIS_WHEEL_NUM] = { -1.0f, +1.0f, -1.0f, +1.0f };

void mecanum_inverse(float vx, float vy, float wz, float wheel_rpm[CHASSIS_WHEEL_NUM])
{
    const float roller = (float)CHASSIS_ROLLER_SIGN;
    const float l = CHASSIS_HALF_WHEELBASE_M + CHASSIS_HALF_TRACK_M;
    const float r = CHASSIS_WHEEL_RADIUS_M;
    const float w_lin = wz * l;     /* 旋转项当量线速度 (m/s) */

    for (int i = 0; i < CHASSIS_WHEEL_NUM; i++)
    {
        float lin = vx + roller * K_VY[i] * vy + roller * K_WZ[i] * w_lin;
        wheel_rpm[i] = (lin / r) * CHASSIS_RAD_S_TO_RPM;
    }
}

void mecanum_forward(const float wheel_rpm[CHASSIS_WHEEL_NUM],
                     float *vx, float *vy, float *wz)
{
    const float roller = (float)CHASSIS_ROLLER_SIGN;
    const float l = CHASSIS_HALF_WHEELBASE_M + CHASSIS_HALF_TRACK_M;
    const float r = CHASSIS_WHEEL_RADIUS_M;

    float w[CHASSIS_WHEEL_NUM];
    for (int i = 0; i < CHASSIS_WHEEL_NUM; i++)
    {
        w[i] = wheel_rpm[i] * CHASSIS_RPM_TO_RAD_S;     /* rad/s */
    }

    *vx = r * 0.25f * ( w[CHASSIS_WHEEL_FL] + w[CHASSIS_WHEEL_FR]
                      + w[CHASSIS_WHEEL_BL] + w[CHASSIS_WHEEL_BR]);
    /* 与逆解的 K_VY 共用同一套符号（左移 = 左前/左后往后、右前/右后往前） */
    *vy = roller * r * 0.25f * ( -w[CHASSIS_WHEEL_FL] + w[CHASSIS_WHEEL_FR]
                               + w[CHASSIS_WHEEL_BL] - w[CHASSIS_WHEEL_BR]);
    /* 与逆解的 K_WZ 共用同一套符号（左后右前），否则里程计 yaw 方向会和真实转向相反 */
    *wz = roller * r * 0.25f / l * ( -w[CHASSIS_WHEEL_FL] + w[CHASSIS_WHEEL_FR]
                                     - w[CHASSIS_WHEEL_BL] + w[CHASSIS_WHEEL_BR]);
}
