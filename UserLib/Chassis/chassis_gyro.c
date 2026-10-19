/**
  ******************************************************************************
  * @file    chassis_gyro.c
  * @brief   底盘偏航闭环（陀螺外环）实现
  *
  * 控制律（每个控制拍）：
  *   wz_ref  = 用户指令 / 自旋速率 / 角度环输出
  *   e       = wz_ref − wz_gyro
  *   wz_out  = clamp( wz_ref + Kp·e + Ki·∫e , ±CHASSIS_MAX_WZ_RAD_S )
  *
  * 三个刻意的设计：
  *   1) 只做「修正」不做「替代」：前馈仍是 wz_ref，陀螺只补打滑/擦地的差额，
  *      所以 IMU 挂了也不会突然失控（退回纯运动学）。
  *   2) 斜坡段冻结积分：轮速还在 1500RPM/s 爬坡时误差是「物理上追不上」造成的，
  *      这时候积分只会积出一大坨，等斜坡走完再吐出来 -> 自旋冲过头。
  *   3) 误差大时不积分（I_BAND）：同理，只处理稳态小误差，不做快速收敛。
  *   4) 输出顶限幅时按误差方向拒绝积分（条件积分抗饱和）。
  ******************************************************************************
  */
#include "chassis_gyro.h"

#include "chassis_config.h"
#include "chassis_math.h"
#include "imu.h"

#include <math.h>
#include <stddef.h>

static chassis_gyro_state_t s_g;
static float s_i_term;              /* 积分项 ∫e dt (rad/s·s) */
static uint32_t s_last_sample_cnt;

void chassis_gyro_reset(void)
{
    s_i_term = 0.0f;
    s_g.wz_corr = 0.0f;
    s_g.wz_out = 0.0f;
    s_g.wz_ref = 0.0f;
    s_g.yaw_err = 0.0f;
    imu_reset_yaw(0.0f);
}

void chassis_gyro_reset_correction(void)
{
    s_i_term = 0.0f;
    s_g.wz_corr = 0.0f;
    s_g.wz_out = 0.0f;
    s_g.yaw_err = 0.0f;
}

void chassis_gyro_init(void)
{
    s_g.mode = CHASSIS_GYRO_MODE_OFF;
    s_g.imu_online = false;
    s_g.spin_rate = CHASSIS_SPIN_RATE_RAD_S;
    s_g.yaw_target = 0.0f;
    s_g.stationary = false;
    s_last_sample_cnt = 0;

    chassis_gyro_reset();
    s_g.imu_online = imu_init();
    if (s_g.imu_online)
    {
        s_g.mode = (chassis_gyro_mode_t)CHASSIS_GYRO_MODE_DEFAULT;
    }
}

void chassis_gyro_set_mode(chassis_gyro_mode_t mode)
{
    if (mode != s_g.mode)
    {
        s_i_term = 0.0f;
        s_g.wz_corr = 0.0f;
        s_g.yaw_err = 0.0f;
    }
    s_g.mode = mode;
}

void chassis_gyro_set_spin_rate(float rad_s)
{
    s_g.spin_rate = chassis_clampf(rad_s, -CHASSIS_MAX_WZ_RAD_S, CHASSIS_MAX_WZ_RAD_S);
}

void chassis_gyro_set_yaw_target(float rad)
{
    s_g.yaw_target = rad;
}

float chassis_gyro_update(float wz_user, float dt, bool active,
                          bool ramping, bool wheels_idle)
{
    const imu_state_t *imu = imu_state();
    bool new_sample;

    s_g.imu_online = imu->healthy;      /* 掉线时外环退回纯运动学，不用陈旧数据 */
    s_g.wz_meas = imu->gyro_lpf[2];
    s_g.yaw_imu = imu->yaw_rad;
    new_sample = (imu->sample_count != s_last_sample_cnt);
    s_last_sample_cnt = imu->sample_count;

    /* ---- 不参与：彻底退回纯运动学（IMU 离线 / 非正常模式） ---- */
    if ((!active) || (!s_g.imu_online) || (s_g.mode == CHASSIS_GYRO_MODE_OFF))
    {
        s_i_term = 0.0f;
        s_g.wz_corr = 0.0f;
        s_g.wz_ref = wz_user;
        s_g.wz_out = wz_user;
        s_g.yaw_err = 0.0f;
        s_g.stationary = false;
        return wz_user;
    }

    /* ---- 1. 参考角速度 ---- */
    switch (s_g.mode)
    {
        case CHASSIS_GYRO_MODE_SPIN:
            s_g.wz_ref = s_g.spin_rate;
            s_g.yaw_err = 0.0f;
            break;

        case CHASSIS_GYRO_MODE_YAW_TARGET:
            s_g.yaw_err = chassis_wrap_pi(s_g.yaw_target - imu->yaw_rad);
            s_g.wz_ref = chassis_clampf(CHASSIS_GYRO_KP_ANGLE * s_g.yaw_err,
                                        -CHASSIS_MAX_WZ_RAD_S * 0.8f,
                                        CHASSIS_MAX_WZ_RAD_S * 0.8f);
            break;

        case CHASSIS_GYRO_MODE_RATE:
        default:
            s_g.wz_ref = wz_user;
            s_g.yaw_err = 0.0f;
            break;
    }

    /* ---- 2. 静止零偏慢速跟踪（只在拿到新样本时推一次） ---- */
    {
        bool stationary = wheels_idle && new_sample
                          && (fabsf(wz_user) < 0.02f)
                          && (fabsf(imu->gyro_lpf[0]) < CHASSIS_GYRO_STATIC_GYRO)
                          && (fabsf(imu->gyro_lpf[1]) < CHASSIS_GYRO_STATIC_GYRO)
                          && (fabsf(imu->gyro_lpf[2]) < CHASSIS_GYRO_STATIC_GYRO);
        s_g.stationary = stationary;
        if (stationary)
        {
            imu_track_bias(true, imu->dt_s);
        }
    }

    /* ---- 3. PI 修正 ---- */
    {
        float e = s_g.wz_ref - s_g.wz_meas;
        float corr;
        float out;
        bool sat = false;
        bool windup;

        corr = (CHASSIS_GYRO_KP * e) + (CHASSIS_GYRO_KI * s_i_term);
        corr = chassis_clampf(corr, -CHASSIS_GYRO_CORR_MAX, CHASSIS_GYRO_CORR_MAX);

        /* 斜坡段 / 误差超出「能力带」时冻结修正量：
         * 起步那一瞬间 e 很大，P 项会直接把 wz 顶到限幅，等于把所有加速度都堆在
         * 最前面（实测仿真：无打滑时自旋峰值冲到 4.29rad/s，超调 23%）。
         * 这类大误差是「物理上还没追上」，交给轮速斜坡和前馈去处理，
         * 外环沿用上一拍的修正值（它代表上一个稳态的打滑补偿）。 */
        if (ramping || (fabsf(e) > CHASSIS_GYRO_I_BAND))
        {
            corr = s_g.wz_corr;
        }

        out = s_g.wz_ref + corr;
        if (out > CHASSIS_GYRO_WZ_LIMIT)
        {
            out = CHASSIS_GYRO_WZ_LIMIT;
            sat = true;
        }
        else if (out < -CHASSIS_GYRO_WZ_LIMIT)
        {
            out = -CHASSIS_GYRO_WZ_LIMIT;
            sat = true;
        }

        /* 零指令 + 车体没转：积分项缓慢回零（τ = 0.3 s）。
         *
         * 没有这一条时的坑（2026-10-18 实测）：架空时给过自旋指令（wheels 实验里
         * wz=2.0），轮子转但车体不转，陀螺读到 0，误差 e 恒为 2.0 -> 积分充满；
         * 指令归零后 i_term 没人清，输出 wz = KI*i_term ≈ 0.1 rad/s 一直挂着，
         * 表现成「没给指令轮子却按自旋模式转」。地板上会自愈（车真的转，误差变负），
         * 所以只在架空/打滑时暴露，很难想明白。 */
        if ((fabsf(s_g.wz_ref) < 0.02f) &&
            (fabsf(s_g.wz_meas) < CHASSIS_GYRO_STATIC_GYRO))
        {
            s_i_term *= (1.0f - (dt / 0.3f));
        }

        /* 条件积分：斜坡中 / 误差太大 / 顶限幅还想往前推 都不积分 */
        windup = sat && (((out > 0.0f) && (e > 0.0f)) || ((out < 0.0f) && (e < 0.0f)));
        if ((!ramping) && (!windup) && (fabsf(e) < CHASSIS_GYRO_I_BAND))
        {
            s_i_term += e * dt;
            s_i_term = chassis_clampf(s_i_term, -CHASSIS_GYRO_I_MAX, CHASSIS_GYRO_I_MAX);
        }

        s_g.wz_corr = corr;
        s_g.wz_out = out;
    }

    return s_g.wz_out;
}

const chassis_gyro_state_t *chassis_gyro_state(void)
{
    return &s_g;
}
