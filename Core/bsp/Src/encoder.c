#include "encoder.h"

/* ------------------------------------------------------------------ */
/* 底层硬件读取                                                         */
/* ------------------------------------------------------------------ */

/* 读取定时器计数值并清零，返回带符号的脉冲增量（取反是因为编码器模式下计数方向与期望相反） */
static int16_t Encoder_ReadDelta(TIM_HandleTypeDef *htim)
{
  int16_t temp = (int16_t)__HAL_TIM_GET_COUNTER(htim);
  __HAL_TIM_SET_COUNTER(htim, 0U);
  return (int16_t)(-temp);
}

int16_t Encoder_GetLeftRawCount(const Encoder_HandleTypeDef *henc)
{
  return (int16_t)__HAL_TIM_GET_COUNTER(henc->left_htim);
}

int16_t Encoder_GetRightRawCount(const Encoder_HandleTypeDef *henc)
{
  return (int16_t)__HAL_TIM_GET_COUNTER(henc->right_htim);
}

/* ------------------------------------------------------------------ */
/* 初始化 / 复位                                                        */
/* ------------------------------------------------------------------ */

void Encoder_Init(Encoder_HandleTypeDef *henc)
{
  HAL_TIM_Encoder_Start(henc->left_htim,  TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(henc->right_htim, TIM_CHANNEL_ALL);
  Encoder_Reset(henc);
}

void Encoder_Reset(Encoder_HandleTypeDef *henc)
{
  __HAL_TIM_SET_COUNTER(henc->left_htim,  0U);
  __HAL_TIM_SET_COUNTER(henc->right_htim, 0U);
  henc->sample.left_delta_count  = 0;
  henc->sample.right_delta_count = 0;
  henc->sample.left_speed_rps    = 0.0f;
  henc->sample.right_speed_rps   = 0.0f;
}

/* ------------------------------------------------------------------ */
/* 运行时更新（在定时器中断或主循环中周期调用）                           */
/* ------------------------------------------------------------------ */

void Encoder_Update(Encoder_HandleTypeDef *henc)
{
  /* 读取本周期脉冲增量并应用极性修正 */
  henc->sample.left_delta_count  = (int16_t)(henc->left_polarity  * Encoder_ReadDelta(henc->left_htim));
  henc->sample.right_delta_count = (int16_t)(henc->right_polarity * Encoder_ReadDelta(henc->right_htim));

  /* 脉冲增量 / 每转脉冲数 / 采样时间 = 转速（转/秒） */
  float left_rps  = (float)henc->sample.left_delta_count  / henc->pulses_per_rev / henc->sample_time_s;
  float right_rps = (float)henc->sample.right_delta_count / henc->pulses_per_rev / henc->sample_time_s;

  /* 一阶低通滤波：filtered += alpha * (new - filtered) */
  henc->sample.left_speed_rps  += henc->filter_alpha * (left_rps  - henc->sample.left_speed_rps);
  henc->sample.right_speed_rps += henc->filter_alpha * (right_rps - henc->sample.right_speed_rps);
}

/* ------------------------------------------------------------------ */
/* 顶层查询接口                                                          */
/* ------------------------------------------------------------------ */

int16_t Encoder_GetLeftDeltaCount(const Encoder_HandleTypeDef *henc)
{
  return henc->sample.left_delta_count;
}

int16_t Encoder_GetRightDeltaCount(const Encoder_HandleTypeDef *henc)
{
  return henc->sample.right_delta_count;
}

float Encoder_GetLeftSpeedRps(const Encoder_HandleTypeDef *henc)
{
  return henc->sample.left_speed_rps;
}

float Encoder_GetRightSpeedRps(const Encoder_HandleTypeDef *henc)
{
  return henc->sample.right_speed_rps;
}

Encoder_SpeedSample Encoder_GetSpeedSample(const Encoder_HandleTypeDef *henc)
{
  return henc->sample;
}
