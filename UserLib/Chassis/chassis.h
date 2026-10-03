/**
  ******************************************************************************
  * @file    chassis.h
  * @brief   麦克纳姆轮底盘主模块：指令入口 + 1kHz 控制循环 + 里程计
  *
  * 使用方式（裸机）：
  *   chassis_init();
  *   while (1) {
  *       chassis_poll();        // 内部按 1kHz tick 执行控制循环
  *       ... 其他应用逻辑 ...
  *   }
  *
  * 控制链：指令速度 -> 运动学逆解 -> 轮速斜坡 -> 速度环 PID -> TB6612
  * 反馈链：编码器 -> 滑动窗口测速 -> 运动学正解 -> 里程计
  ******************************************************************************
  */
#ifndef USERLIB_CHASSIS_CHASSIS_H
#define USERLIB_CHASSIS_CHASSIS_H

#include "mecanum.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    /* ---- 指令（用户通过 chassis_set_velocity 写入） ---- */
    float cmd_vx;   /* m/s，+x 前进 */
    float cmd_vy;   /* m/s，+y 左移 */
    float cmd_wz;   /* rad/s，逆时针为正 */

    /* ---- 测量 ---- */
    float meas_vx, meas_vy, meas_wz;                    /* 正解得到的车体速度 */
    float wheel_rpm_ref[CHASSIS_WHEEL_NUM];             /* 斜坡后的目标轮速 (RPM) */
    float wheel_rpm_meas[CHASSIS_WHEEL_NUM];            /* 实测轮速 (RPM，已方向归一) */
    float wheel_duty[CHASSIS_WHEEL_NUM];                /* 各轮输出占空比（千分比） */
    int32_t wheel_count[CHASSIS_WHEEL_NUM];             /* 各轮编码器累计计数（原始方向） */

    /* ---- 里程计（相对上电位姿） ---- */
    float x, y;     /* m */
    float yaw;      /* rad，逆时针为正 */
} chassis_state_t;

/** 初始化：绑定电机/编码器、启动 PWM 与编码器、置高 STBY */
void chassis_init(void);

/**
 * @brief 主循环高频调用；内部检测 1kHz tick，到点执行一次控制循环。
 *        调用得越勤，节拍越准；未到 tick 时立即返回。
 */
void chassis_poll(void);

/** 设定车体目标速度，自动限幅 */
void chassis_set_velocity(float vx, float vy, float wz);

/** 停止（目标速度清零，经斜坡减速） */
void chassis_stop(void);

/** 读取底盘状态（测量值/里程计，只读） */
const chassis_state_t *chassis_state(void);

/** 底盘库时钟 (ms)，基于 HAL_GetTick，1kHz */
uint32_t chassis_tick_ms(void);

#endif /* USERLIB_CHASSIS_CHASSIS_H */
