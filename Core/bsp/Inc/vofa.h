#ifndef __VOFA_H
#define __VOFA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "usart.h"

#define VOFA_MAX_CHANNELS  10

void VOFA_Send(float *data, uint8_t count);

#ifdef __cplusplus
}
#endif

#endif
