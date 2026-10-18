/**
  ******************************************************************************
  * @file    chassis_tune.h
  * @brief   PID 整定实验模块：开环静态标定表 / 闭环阶跃响应采集
  *
  * 只在整定期间使用（CHASSIS_TUNE_ENABLE=1）。正式跑车时置 0，
  * 本模块整体不编译，对底盘功能零影响。
  *
  * 用法（main.c 里）：
  *   chassis_init();
  *   chassis_tune_init();
  *   while (1) { chassis_poll(); chassis_tune_poll(); }
  *
  * 实验流程自上电自动开始（1s 后），数据经 USART1 输出：
  *   实验 1（开环静态标定）：文本，每行一个占空比档位的四轮稳态转速
  *   实验 2（闭环阶跃）：二进制 int16 采样块，配 tools/ 下的分析脚本用
  ******************************************************************************
  */
#ifndef USERLIB_CHASSIS_CHASSIS_TUNE_H
#define USERLIB_CHASSIS_CHASSIS_TUNE_H

#include "chassis.h"

/** 初始化实验（必须在 chassis_init() 之后调用） */
void chassis_tune_init(void);

/** 1kHz 控制循环回调（由 chassis_update 在每周期末尾调用） */
void chassis_tune_on_tick(const chassis_state_t *st);

/** 主循环调用：实验结束后负责把数据从串口吐出去 */
void chassis_tune_poll(void);

/** 实验是否还没结束（调试输出用它让路） */
int chassis_tune_busy(void);

#endif /* USERLIB_CHASSIS_CHASSIS_TUNE_H */
