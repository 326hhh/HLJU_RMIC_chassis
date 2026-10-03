/**
  ******************************************************************************
  * @file    mecanum.c
  * @brief   麦克纳姆轮运动学实现
  *
  * 逆解公式（推导见 docs/CHASSIS_LIBRARY.md 附录）：
  *   omega_i = ( vx + S*ky_i*vy + S*kw_i*wz*L ) / r
  *   其中 L = lx + ly（半轴距+半轮距），S = CHASSIS_ROLLER_SIGN
  *   ky = {+1,-1,-1,+1},  kw = {-1,-1,+1,+1}  （FL FR BL BR 顺序）
  ******************************************************************************
  */
#include "mecanum.h"
#include "chassis_config.h"
#include "chassis_math.h"

/* 每轮 vy / wz 项的符号系数（FL FR BL BR） */
static const float K_VY[CHASSIS_WHEEL_NUM] = { +1.0f, -1.0f, -1.0f, +1.0f };
static const float K_WZ[CHASSIS_WHEEL_NUM] = { -1.0f, -1.0f, +1.0f, +1.0f };

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
    *vy = roller * r * 0.25f * ( w[CHASSIS_WHEEL_FL] - w[CHASSIS_WHEEL_FR]
                               - w[CHASSIS_WHEEL_BL] + w[CHASSIS_WHEEL_BR]);
    *wz = roller * r * 0.25f / l * ( -w[CHASSIS_WHEEL_FL] - w[CHASSIS_WHEEL_FR]
                                     + w[CHASSIS_WHEEL_BL] + w[CHASSIS_WHEEL_BR]);
}
