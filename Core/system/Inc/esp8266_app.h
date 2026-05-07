#ifndef __ESP8266_APP_H__
#define __ESP8266_APP_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>

typedef enum
{
  ESP8266_STATE_IDLE = 0,
  ESP8266_STATE_A1,
  ESP8266_STATE_A2,
  ESP8266_STATE_A3,
  ESP8266_STATE_B1,
  ESP8266_STATE_B2,
  ESP8266_STATE_B3
} ESP8266_CommandState;

void ESP8266_App_Init(UART_HandleTypeDef *huart);
void ESP8266_App_Run(void);

/*
 * 将指令解码为两个路口的转向方向
 *   字母：A → dir1=-1（左转），B → dir1=+1（右转）
 *   数字：1 → dir2=+1（右转），2 → dir2=0（直行），3 → dir2=-1（左转）
 * state=IDLE 时两者均置 0
 */
void ESP8266_App_DecodeDirs(ESP8266_CommandState state, int8_t *dir1, int8_t *dir2);
bool ESP8266_App_IsReady(void);
bool ESP8266_App_PopCommand(ESP8266_CommandState *out_command);
ESP8266_CommandState ESP8266_App_GetState(void);
const char *ESP8266_App_GetStateName(void);

void ESP8266_App_UartRxCpltCallback(UART_HandleTypeDef *huart);
void ESP8266_App_UartTxCpltCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif