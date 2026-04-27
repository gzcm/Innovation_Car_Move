#ifndef __TB6612_H
#define __TB6612_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TB6612_PWM_MAX_PERCENT  (100)

typedef enum
{
    TB6612_MOTOR_LEFT = 0,
    TB6612_MOTOR_RIGHT
} TB6612_MotorId;

/* 单个电机的硬件配置 */
typedef struct
{
    GPIO_TypeDef *in1_port;         /* 方向引脚 IN1 所在 GPIO 端口 */
    uint16_t      in1_pin;          /* 方向引脚 IN1 引脚号 */
    GPIO_TypeDef *in2_port;         /* 方向引脚 IN2 所在 GPIO 端口 */
    uint16_t      in2_pin;          /* 方向引脚 IN2 引脚号 */
    uint32_t      pwm_channel;      /* 对应定时器 PWM 通道，如 TIM_CHANNEL_1 */
    int8_t        forward_polarity; /* 正向极性：接线正向为 1，反向接线为 -1 */
} TB6612_MotorConfig;

/* TB6612 设备句柄 */
typedef struct
{
    TIM_HandleTypeDef *htim;  /* PWM 定时器句柄 */
    TB6612_MotorConfig left;  /* 左电机配置 */
    TB6612_MotorConfig right; /* 右电机配置 */
} TB6612_HandleTypeDef;

void TB6612_Init(TB6612_HandleTypeDef *hdev);
void TB6612_StopAll(TB6612_HandleTypeDef *hdev);
void TB6612_SetMotor(TB6612_HandleTypeDef *hdev, TB6612_MotorId motor_id, int16_t speed_percent);
void TB6612_SetMotorPair(TB6612_HandleTypeDef *hdev, int16_t left_speed_percent, int16_t right_speed_percent);

#ifdef __cplusplus
}
#endif

#endif
