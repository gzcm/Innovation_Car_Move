#include "vofa.h"
#include <string.h>

/* JustFloat 帧尾：IEEE 754 NaN (小端) */
static const uint8_t VOFA_TAIL[4] = {0x00, 0x00, 0x80, 0x7F};

/*
 * 发送 JustFloat 帧到 Vofa+ 上位机
 * data  : float 数组指针
 * count : 通道数量，不超过 VOFA_MAX_CHANNELS
 */
void VOFA_Send(float *data, uint8_t count)
{
    if (count == 0 || count > VOFA_MAX_CHANNELS) return;

    uint8_t buf[VOFA_MAX_CHANNELS * sizeof(float) + sizeof(VOFA_TAIL)];
    uint16_t data_len = count * sizeof(float);

    memcpy(buf, data, data_len);
    memcpy(buf + data_len, VOFA_TAIL, sizeof(VOFA_TAIL));

    HAL_UART_Transmit(&huart1, buf, data_len + sizeof(VOFA_TAIL), 100);
}
