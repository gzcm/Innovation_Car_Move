#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 单次采样结果 */
typedef struct
{
  int16_t left_delta_count;  /* 左轮本周期脉冲增量 */
  int16_t right_delta_count; /* 右轮本周期脉冲增量 */
  float   left_speed_rps;    /* 左轮滤波后转速（转/秒） */
  float   right_speed_rps;   /* 右轮滤波后转速（转/秒） */
} Encoder_SpeedSample;

/* 编码器设备句柄，调用方负责声明并初始化硬件配置字段 */
typedef struct
{
  TIM_HandleTypeDef *left_htim;        /* 左轮编码器定时器 */
  TIM_HandleTypeDef *right_htim;       /* 右轮编码器定时器 */
  int8_t             left_polarity;    /* 左轮极性：接线正向为 1，反向为 -1 */
  int8_t             right_polarity;   /* 右轮极性：接线正向为 1，反向为 -1 */
  float              sample_time_s;    /* 采样周期（秒），需与调用 Encoder_Update 的定时器一致 */
  float              pulses_per_rev;   /* 电机每转脉冲数（减速箱输出轴） */
  float              filter_alpha;     /* 一阶低通滤波系数，越小越平滑但响应越慢 */
  Encoder_SpeedSample sample;          /* 当前采样结果（内部维护，只读） */
} Encoder_HandleTypeDef;

void    Encoder_Init(Encoder_HandleTypeDef *henc);
void    Encoder_Update(Encoder_HandleTypeDef *henc);
void    Encoder_Reset(Encoder_HandleTypeDef *henc);

int16_t Encoder_GetLeftDeltaCount(const Encoder_HandleTypeDef *henc);
int16_t Encoder_GetRightDeltaCount(const Encoder_HandleTypeDef *henc);
int16_t Encoder_GetLeftRawCount(const Encoder_HandleTypeDef *henc);
int16_t Encoder_GetRightRawCount(const Encoder_HandleTypeDef *henc);
float   Encoder_GetLeftSpeedRps(const Encoder_HandleTypeDef *henc);
float   Encoder_GetRightSpeedRps(const Encoder_HandleTypeDef *henc);
Encoder_SpeedSample Encoder_GetSpeedSample(const Encoder_HandleTypeDef *henc);

#ifdef __cplusplus
}
#endif

#endif
