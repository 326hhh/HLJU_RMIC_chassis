/**
  ******************************************************************************
  * @file    motor.h
  * @brief   TB6612 单路电机抽象：占空比输入，内部处理方向引脚电平
  *
  * 占空比范围 [-1000, +1000]（千分比），正负代表正反转，0 为滑行停止。
  ******************************************************************************
  */
#ifndef USERLIB_CHASSIS_MOTOR_H
#define USERLIB_CHASSIS_MOTOR_H

#include "stm32f4xx_hal.h"

typedef struct
{
    TIM_HandleTypeDef *htim;        /* PWM 定时器 */
    uint32_t channel;               /* PWM 通道 */
    GPIO_TypeDef *in1_port;         /* TB6612 IN1 */
    uint16_t in1_pin;
    GPIO_TypeDef *in2_port;         /* TB6612 IN2 */
    uint16_t in2_pin;
    GPIO_PinState fwd_in1;          /* "正转"时 IN1 的电平（与 in2 配合定义正方向） */
    GPIO_PinState fwd_in2;          /* "正转"时 IN2 的电平 */
    float duty;                     /* 当前占空比（千分比，带符号） */
} motor_t;

/**
 * @brief 初始化并启动一路电机（会调用 HAL_TIM_PWM_Start 并清零输出）
 */
void motor_init(motor_t *motor, TIM_HandleTypeDef *htim, uint32_t channel,
                GPIO_TypeDef *in1_port, uint16_t in1_pin,
                GPIO_TypeDef *in2_port, uint16_t in2_pin,
                GPIO_PinState fwd_in1, GPIO_PinState fwd_in2);

/**
 * @brief 设置占空比 [-1000, +1000]，自动限幅
 */
void motor_set_duty(motor_t *motor, float duty);

/** 读取当前占空比 */
float motor_get_duty(const motor_t *motor);

#endif /* USERLIB_CHASSIS_MOTOR_H */
