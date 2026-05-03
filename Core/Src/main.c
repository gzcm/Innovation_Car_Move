/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include "tb6612.h"
#include "button.h"
#include "encoder.h"
#include "pid.h"
#include "vofa.h"
#include "track.h"
#include "tracker.h"
#include "HWT101.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ENCODER_SAMPLE_TIME_S               (0.02f)
#define ENCODER_PULSES_PER_REVOLUTION       (2040.0f)
#define ENCODER_LEFT_POLARITY               (1)
#define ENCODER_RIGHT_POLARITY              (1)
#define ENCODER_SPEED_FILTER_ALPHA          (0.3f)

/* 速度 PI 初始参数（根据开环测量整定，后续按实测调整） */
#define PID_KFF_LEFT    (24.8f)   /* 1 / 0.0403 rps/% */
#define PID_KFF_RIGHT   (22.8f)   /* 1 / 0.0438 rps/% */
#define PID_KP          (25.0f)
#define PID_KI          (120.0f)  /* Kp/Ti，Ti ≈ 0.2s */

#define TRACKER_K_TRACK  (0.6f)   /* 循迹增益，position → 差速转换 */
/* 目标转速档位（rps），按键逐档切换 */
static const float RPS_STEPS[] = { 0.0f, 0.5f, 1.0f, 1.5f, 2.0f, 2.5f };
#define RPS_STEPS_LEN  (sizeof(RPS_STEPS) / sizeof(RPS_STEPS[0]))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static TB6612_HandleTypeDef hTB6612 = {
    .htim             = &htim3,
    .left = {
		.in1_port        = MOTOR_L_IN1_GPIO_Port,
		.in1_pin         = MOTOR_L_IN1_Pin,
		.in2_port        = MOTOR_L_IN2_GPIO_Port,
		.in2_pin         = MOTOR_L_IN2_Pin,
		.pwm_channel     = TIM_CHANNEL_2,
		.forward_polarity = 1,
    },
    .right = {
		.in1_port        = MOTOR_R_IN1_GPIO_Port,
		.in1_pin         = MOTOR_R_IN1_Pin,
		.in2_port        = MOTOR_R_IN2_GPIO_Port,
		.in2_pin         = MOTOR_R_IN2_Pin,
		.pwm_channel     = TIM_CHANNEL_1,
		.forward_polarity = 1,
    },
};

/* 编码器句柄：TIM2左轮，TIM4右轮，TIM1提供50Hz采样中断 */
static Encoder_HandleTypeDef hEncoder = {
    .left_htim      = &htim2,
    .right_htim     = &htim4,
    .left_polarity  = ENCODER_LEFT_POLARITY,
    .right_polarity = ENCODER_RIGHT_POLARITY,
    .sample_time_s  = ENCODER_SAMPLE_TIME_S,
    .pulses_per_rev = ENCODER_PULSES_PER_REVOLUTION,
    .filter_alpha   = ENCODER_SPEED_FILTER_ALPHA,
};

static SpeedPI_t hPidLeft;
static SpeedPI_t hPidRight;
/* 目标转速，主循环写入，TIM1 中断读取 */
static volatile float target_rps = 0.0f;
/* VOFA 发送标志，TIM1 中断置位，主循环发送 */
static volatile uint8_t vofa_pending = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_UART5_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init(&hi2c2);
  TB6612_Init(&hTB6612);
  Encoder_Init(&hEncoder);
  SpeedPI_Init(&hPidLeft,  PID_KFF_LEFT,  PID_KP, PID_KI, ENCODER_SAMPLE_TIME_S);
  SpeedPI_Init(&hPidRight, PID_KFF_RIGHT, PID_KP, PID_KI, ENCODER_SAMPLE_TIME_S);
  Tracker_Init(RPS_STEPS[1], TRACKER_K_TRACK);  /* 初始基础速度 0.5 rps */
  HWT101_init(&huart2);    
  HAL_TIM_Base_Start_IT(&htim1);
  OLED_ShowString(1, 1, "Speed PID Ctrl  ");
  OLED_ShowString(2, 1, "Press btn start ");
  OLED_ShowString(3, 1, "                ");
  OLED_ShowString(4, 1, "                ");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HWT101_Update();       

	  /* 按钮逐档切换目标转速：0 → 0.5 → 1.0 → 1.5 → 2.0 → 2.5 → 0 → ... */
	  if (Button_CheckToggleRequest())
	  {
		  static uint8_t step = 0;
		  step = (step + 1) % RPS_STEPS_LEN;
		  target_rps = RPS_STEPS[step];
		  Tracker_SetSpeed(target_rps);
		  if (target_rps == 0.0f) {
			  SpeedPI_Reset(&hPidLeft);
			  SpeedPI_Reset(&hPidRight);
			  TB6612_StopAll(&hTB6612);
		  }
	  }

	  /* VOFA 数据上发（由 TIM1 ISR 触发，约 50Hz） */
	  if (vofa_pending) {
		  vofa_pending = 0;
		  Encoder_SpeedSample vofa_spd = Encoder_GetSpeedSample(&hEncoder);
		  float vofa_pos = Track_GetPosition();
		  float vofa_tgt_l, vofa_tgt_r;
		  Tracker_Update(vofa_pos, &vofa_tgt_l, &vofa_tgt_r);
		  float vofa_data[8] = {
		      target_rps,
		      vofa_pos,
		      vofa_tgt_l,
		      vofa_tgt_r,
		      vofa_spd.left_speed_rps,
		      vofa_spd.right_speed_rps,
		      hPidLeft.output,
		      hPidRight.output
		  };
		  VOFA_Send(vofa_data, 8);
	  }

	  /* OLED 刷新（约 5Hz） */
	  {
		  static uint32_t last_tick = 0;
		  static char disp_buf[17];
		  if (HAL_GetTick() - last_tick >= 200)
		  {
			  last_tick = HAL_GetTick();
			  const HWT101_Data_t *imu = HWT101_GetData();

			  snprintf(disp_buf, sizeof(disp_buf), "Tgt:%+6.2f rps  ", (double)target_rps);
			  OLED_ShowString(1, 1, disp_buf);

			  snprintf(disp_buf, sizeof(disp_buf), "Yaw:%+7.1f deg  ", (double)imu->yaw);
			  OLED_ShowString(2, 1, disp_buf);

			  snprintf(disp_buf, sizeof(disp_buf), "Gz: %+7.1f d/s  ", (double)imu->gyroZ);
			  OLED_ShowString(3, 1, disp_buf);
		  }
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* TIM1 周期中断回调，每 20ms 更新编码器并执行速度闭环 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) {
        Encoder_Update(&hEncoder);

        float tgt = target_rps;
        if (tgt == 0.0f) {
            return;
        }

        /* 循迹差速：线偏移 → 左右速度目标 */
        float position = Track_GetPosition();
        float tgt_l, tgt_r;
        Tracker_Update(position, &tgt_l, &tgt_r);

        Encoder_SpeedSample spd = Encoder_GetSpeedSample(&hEncoder);
        int16_t pwm_l = (int16_t)SpeedPI_Update(&hPidLeft,  tgt_l, spd.left_speed_rps);
        int16_t pwm_r = (int16_t)SpeedPI_Update(&hPidRight, tgt_r, spd.right_speed_rps);
        TB6612_SetMotorPair(&hTB6612, pwm_l, pwm_r);
        vofa_pending = 1;
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
