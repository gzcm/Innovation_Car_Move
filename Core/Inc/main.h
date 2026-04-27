/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define KEY1_Pin GPIO_PIN_0
#define KEY1_GPIO_Port GPIOC
#define ENCODER_L_B_Pin GPIO_PIN_0
#define ENCODER_L_B_GPIO_Port GPIOA
#define ENCODER_L_A_Pin GPIO_PIN_1
#define ENCODER_L_A_GPIO_Port GPIOA
#define ENCODER_R_A_Pin GPIO_PIN_12
#define ENCODER_R_A_GPIO_Port GPIOD
#define ENCODER_R_B_Pin GPIO_PIN_13
#define ENCODER_R_B_GPIO_Port GPIOD
#define PWM_R_Pin GPIO_PIN_6
#define PWM_R_GPIO_Port GPIOC
#define PWM_L_Pin GPIO_PIN_7
#define PWM_L_GPIO_Port GPIOC
#define MOTOR_R_IN2_Pin GPIO_PIN_8
#define MOTOR_R_IN2_GPIO_Port GPIOC
#define MOTOR_L_IN2_Pin GPIO_PIN_9
#define MOTOR_L_IN2_GPIO_Port GPIOC
#define MOTOR_R_IN1_Pin GPIO_PIN_8
#define MOTOR_R_IN1_GPIO_Port GPIOA
#define MOTOR_L_IN1_Pin GPIO_PIN_15
#define MOTOR_L_IN1_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
