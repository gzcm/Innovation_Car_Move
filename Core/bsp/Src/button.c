#include "button.h"

#define BUTTON_DEBOUNCE_MS  (10U)

uint8_t Button_CheckToggleRequest(void)
{
  uint8_t toggle_requested = 0;

  if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET)
  {
    HAL_Delay(BUTTON_DEBOUNCE_MS);

    if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET)
    {
      toggle_requested = 1;
    }

    while (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET);
  }

  return toggle_requested;
}
