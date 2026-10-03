/**
  ******************************************************************************
  * @file    encoder.c
  * @brief   编码器抽象实现
  ******************************************************************************
  */
#include "encoder.h"

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
