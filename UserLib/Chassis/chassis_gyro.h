/**
  ******************************************************************************
  * @file    chassis_gyro.h
  * @brief   基于 IMU 的底盘偏航闭环：小陀螺自旋 / 云台跟随
  *
  * 为什么需要它：
  *   纯运动学（指令 -> 轮速 -> 麦轮）假设轮子不打滑、麦轮横向摩擦无限大。
  *   原地自旋时四个轮子都在横向擦地，实际角速度和指令能差 10~30%，
  *   而且换电池/换地面就变。用陀螺实测角速度做外环修正，自旋速率才稳得住。
  *
  * 三种模式：
  *   OFF       ：不参与，wz 直接用用户指令（出问题时的退路）
  *   RATE      ：角速度闭环，用户给的 wz 当目标，陀螺只做修正
  *                —— 正常跑车/平移/自旋都用它
  *   SPIN      ：小陀螺，恒定自旋速率（chassis_gyro_set_spin_rate）
  *   YAW_TARGET：底盘 yaw 追一个目标角（chassis_gyro_set_yaw_target）
  *                —— 预留给「云台跟随」：云台世界角度一给，底盘自己转过去对准
  ******************************************************************************
  */
#ifndef USERLIB_CHASSIS_CHASSIS_GYRO_H
#define USERLIB_CHASSIS_CHASSIS_GYRO_H

#include <stdbool.h>

typedef enum
{
    CHASSIS_GYRO_MODE_OFF = 0,      /* 不参与 */
    CHASSIS_GYRO_MODE_RATE,         /* 角速度闭环（默认） */
    CHASSIS_GYRO_MODE_SPIN,         /* 小陀螺：恒定自旋 */
    CHASSIS_GYRO_MODE_YAW_TARGET,   /* 偏航角闭环：底盘对准目标角 */
} chassis_gyro_mode_t;

typedef struct
{
    chassis_gyro_mode_t mode;
    bool  imu_online;
    float wz_ref;       /* 本次参考角速度 (rad/s) */
    float wz_meas;      /* 陀螺实测角速度 (rad/s) */
    float wz_corr;      /* PI 修正量 (rad/s) */
    float wz_out;       /* 实际下发给运动学的 wz (rad/s) */
    float yaw_imu;      /* 陀螺积分 yaw (rad, -PI..PI) */
    float yaw_err;      /* 角度模式下的目标误差 (rad) */
    float spin_rate;    /* 自旋模式目标速率 (rad/s) */
    float yaw_target;   /* 角度模式目标角 (rad) */
    bool  stationary;   /* 外部判断的静止标志 */
} chassis_gyro_state_t;

/** 初始化：调用 imu_init()，复位内部状态（在 chassis_init 里调用） */
void chassis_gyro_init(void);

/**
 * @brief 1kHz 控制环每拍调用
 * @param wz_user     用户指令角速度 (rad/s)
 * @param dt          控制周期 (s)
 * @param active      true = 允许陀螺环介入（正常模式且电机使能）
 * @param ramping     true = 轮速斜坡还在爬（此时冻结积分，防止积分饱和）
 * @param wheels_idle true = 四个轮子都停着（用于零偏慢速跟踪）
 * @return 最终下发给运动学的 wz (rad/s)
 */
float chassis_gyro_update(float wz_user, float dt, bool active,
                          bool ramping, bool wheels_idle);

/** 读状态（只读） */
const chassis_gyro_state_t *chassis_gyro_state(void);

/** 切模式（会清空积分与角度误差） */
void chassis_gyro_set_mode(chassis_gyro_mode_t mode);

/** SPIN 模式的目标自旋速率 (rad/s)，逆时针为正 */
void chassis_gyro_set_spin_rate(float rad_s);

/** YAW_TARGET 模式的目标偏航角 (rad)，逆时针为正 */
void chassis_gyro_set_yaw_target(float rad);

/** 清积分 / 清角度误差 / 把陀螺积分 yaw 归零 */
void chassis_gyro_reset(void);

/**
 * @brief 只清外环修正量（积分 + 修正输出），**不动 yaw**
 *
 * 停车 / 急停 / 看门狗触发时调用：防止上一次打滑/自旋积下来的修正量
 * 在零指令下继续推着轮子转（架空时尤其明显）。
 */
void chassis_gyro_reset_correction(void);

#endif /* USERLIB_CHASSIS_CHASSIS_GYRO_H */
