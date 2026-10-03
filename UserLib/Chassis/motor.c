/**
  ******************************************************************************
  * @file    motor.c
  * @brief   TB6612 单路电机抽象实现
  ******************************************************************************
  */
#include "motor.h"
#include "chassis_math.h"

#include <math.h>

void motor_init(motor_t *motor, TIM_HandleTypeDef *htim, uint32_t channel,
                GPIO_TypeDef *in1_port, uint16_t in1_pin,
                GPIO_TypeDef *in2_port, uint16_t in2_pin,
                GPIO_PinState fwd_in1, GPIO_PinState fwd_in2)
{
    motor->htim = htim;
    motor->channel = channel;
    motor->in1_port = in1_port;
    motor->in1_pin = in1_pin;
    motor->in2_port = in2_port;
    motor->in2_pin = in2_pin;
    motor->fwd_in1 = fwd_in1;
    motor->fwd_in2 = fwd_in2;
    motor->duty = 0.0f;

    HAL_TIM_PWM_Start(htim, channel);
    HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(htim, channel, 0);
}

void motor_set_duty(motor_t *motor, float duty)
{
    duty = chassis_clampf(duty, -1000.0f, 1000.0f);

    /* 零占空比 -> 滑行停止（TB6612 的 IN1=IN2=0） */
    if (fabsf(duty) < 0.5f)
    {
        HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, 0);
        motor->duty = 0.0f;
        return;
    }

    GPIO_PinState in1 = (duty > 0.0f) ? motor->fwd_in1
                                      : (GPIO_PinState)(motor->fwd_in1 ^ 1u);
    GPIO_PinState in2 = (duty > 0.0f) ? motor->fwd_in2
                                      : (GPIO_PinState)(motor->fwd_in2 ^ 1u);

    HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, in1);
    HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, in2);

    uint32_t period = motor->htim->Init.Period + 1u;
    uint32_t compare = (uint32_t)(fabsf(duty) * 0.001f * (float)period + 0.5f);
    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, compare);

    motor->duty = duty;
}

float motor_get_duty(const motor_t *motor)
{
    return motor->duty;
}
