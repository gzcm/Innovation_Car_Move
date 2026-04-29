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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include "tb6612.h"
#include "button.h"
#include "encoder.h"
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
  /* USER CODE BEGIN 2 */
  OLED_Init(&hi2c2);
  TB6612_Init(&hTB6612);
  OLED_ShowString(1, 1, "TB6612 Init!");
  Encoder_Init(&hEncoder);
  HAL_TIM_Base_Start_IT(&htim1);   /* 启动TIM1，50Hz中断触发 Encoder_Update */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* ====== 编码器速度显示测试 ====== */
	  {
		  static uint32_t last_tick = 0;
		  static char disp_buf[17];
		  if (HAL_GetTick() - last_tick >= 200) {
			  last_tick = HAL_GetTick();
			  Encoder_SpeedSample spd = Encoder_GetSpeedSample(&hEncoder);

			  snprintf(disp_buf, sizeof(disp_buf), "L:%5d %5.1f rps",
					  spd.left_delta_count, (double)spd.left_speed_rps);
			  OLED_ShowString(2, 1, disp_buf);

			  snprintf(disp_buf, sizeof(disp_buf), "R:%5d %5.1f rps",
					  spd.right_delta_count, (double)spd.right_speed_rps);
			  OLED_ShowString(3, 1, disp_buf);
		  }
	  }
	  /* ================================= */

	  if(Button_CheckToggleRequest()){
		  TB6612_SetMotorPair(&hTB6612, -25, -25);

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

/* TIM1 周期中断回调，每20ms调用一次编码器更新 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) {
        Encoder_Update(&hEncoder);
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
