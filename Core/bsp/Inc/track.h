#ifndef __TRACK_H
#define __TRACK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define TRACK_COUNT  4

/* 线检测电平：黑线=0 或 黑线=1，根据传感器模块调整 */
#define TRACK_BLACK_LEVEL  0

typedef struct {
    uint8_t raw[TRACK_COUNT];  /* 原始电平（0/1） */
    float   position;           /* 线位置，负=左 正=右 0=正中 */
    uint8_t online;            /* 1=检测到线，0=全线丢失 */
} Track_Data;

void        Track_Init(void);
Track_Data  Track_Read(void);
float       Track_GetPosition(void);

#ifdef __cplusplus
}
#endif

#endif
