#ifndef __APP_H
#define __APP_H

void App_Init(void);      /* 所有外设与控制层初始化 */
void App_Update(void);    /* 主循环体：按钮 / VOFA / OLED */
void App_TimerISR(void);  /* TIM1 中断体：控制环 */

#endif
