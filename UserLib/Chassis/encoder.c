/**
  ******************************************************************************
  * @file    encoder.c
  * @brief   编码器抽象实现
  ******************************************************************************
  */
#include "encoder.h"

/* 编码器输入数字滤波强度（0=不滤波）。8 = f_DTS/256 采样、连续 8 个一致才认边沿，
 * 约等于十几微秒的尖峰抑制。电机 PWM 和换相噪声会灌进 A/B 线，不滤波时
 * 会出现"幽灵计数"（实测 FR 一路 8000 采样里 100+ 次物理不可能的跳变，
 * 开滤波后归零）。TIM1 在 APB2(168MHz)、TIM3/4/8 在 APB1(84MHz)，
 * 滤波后采样率仍远高于最高转速下的边沿率，不影响测速。 */
#define ENC_INPUT_FILTER   8u

static void encoder_apply_filter(TIM_TypeDef *tim)
{
    uint32_t ccmr1 = tim->CCMR1;

    ccmr1 &= ~(uint32_t)(TIM_CCMR1_IC1F | TIM_CCMR1_IC2F);
    ccmr1 |= ((uint32_t)ENC_INPUT_FILTER << 4);     /* IC1F: bits 7:4 */
    ccmr1 |= ((uint32_t)ENC_INPUT_FILTER << 12);    /* IC2F: bits 15:12 */
    tim->CCMR1 = ccmr1;
}

void encoder_start(encoder_t *enc, TIM_HandleTypeDef *htim)
{
    enc->htim = htim;
    enc->last_cnt = 0;
    enc->total = 0;
    enc->ring_idx = 0;
    enc->ring_count = 0;
    enc->window_sum = 0;
    for (int i = 0; i < CHASSIS_SPEED_WINDOW_MS; i++)
    {
        enc->ring[i] = 0;
    }

    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
    encoder_apply_filter(htim->Instance);
    enc->last_cnt = (int16_t)(uint16_t)__HAL_TIM_GET_COUNTER(htim);
}

void encoder_update(encoder_t *enc)
{
    int16_t cnt = (int16_t)(uint16_t)__HAL_TIM_GET_COUNTER(enc->htim);

    /* 16 位计数器差值：无符号相减后截断，天然处理溢出回绕
     * （前提：单周期增量 < 32767，1kHz 下远未达到） */
    int16_t delta = (int16_t)((uint16_t)cnt - (uint16_t)enc->last_cnt);
    enc->last_cnt = cnt;
    enc->total += delta;

    /* 滑动窗口：替换最旧样本，增量维护窗口和 */
    enc->window_sum += (int32_t)delta - (int32_t)enc->ring[enc->ring_idx];
    enc->ring[enc->ring_idx] = delta;
    enc->ring_idx = (uint8_t)((enc->ring_idx + 1u) % CHASSIS_SPEED_WINDOW_MS);
    if (enc->ring_count < CHASSIS_SPEED_WINDOW_MS)
    {
        enc->ring_count++;
    }
}

float encoder_speed_rpm(const encoder_t *enc)
{
    if (enc->ring_count == 0)
    {
        return 0.0f;
    }
    float window_s = (float)enc->ring_count * CHASSIS_CONTROL_PERIOD_S;
    return (float)enc->window_sum * (60.0f / (float)CHASSIS_ENCODER_CPR) / window_s;
}

int32_t encoder_total(const encoder_t *enc)
{
    return enc->total;
}
