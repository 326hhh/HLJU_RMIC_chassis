/**
  ******************************************************************************
  * @file    encoder.h
  * @brief   编码器抽象：16 位计数器溢出拼接 + 滑动窗口测速
  ******************************************************************************
  */
#ifndef USERLIB_CHASSIS_ENCODER_H
#define USERLIB_CHASSIS_ENCODER_H

#include "stm32f4xx_hal.h"
#include "chassis_config.h"

#include <stdint.h>

typedef struct
{
    TIM_HandleTypeDef *htim;                          /* 编码器模式定时器 */
    int16_t last_cnt;                                 /* 上次采样计数值 */
    int32_t total;                                    /* 32 位累计计数 */
    int16_t ring[CHASSIS_SPEED_WINDOW_MS];            /* 增量环形缓冲（每控制周期 1 个样本） */
    uint8_t ring_idx;
    uint8_t ring_count;                               /* 当前有效样本数 */
    int32_t window_sum;                               /* 窗口内增量之和 */
} encoder_t;

/**
 * @brief 启动编码器（TIM 编码器模式，x4 计数）
 */
void encoder_start(encoder_t *enc, TIM_HandleTypeDef *htim);

/**
 * @brief 每个控制周期采样一次：更新累计计数与测速窗口
 */
void encoder_update(encoder_t *enc);

/**
 * @brief 返回窗口平均转速 (RPM)，符号与计数方向一致
 */
float encoder_speed_rpm(const encoder_t *enc);

/** 读取累计计数 */
int32_t encoder_total(const encoder_t *enc);

#endif /* USERLIB_CHASSIS_ENCODER_H */
