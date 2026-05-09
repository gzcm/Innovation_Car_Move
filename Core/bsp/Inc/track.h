#ifndef __TRACK_H
#define __TRACK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define TRACK_COUNT  4

/* 线检测电平：黑线=0 或 黑线=1，根据传感器模块调整 */
#define TRACK_BLACK_LEVEL  0

/* 路口"全黑"判定门槛：4 路中至少 N 路读到黑即视为全黑
 * 设为 4 = 严格全黑；设为 3 = 容忍一路漏读（更敏感、响应更快） */
#define TRACK_ALLBLACK_MIN 3

typedef struct {
    uint8_t raw[TRACK_COUNT];  /* 原始电平（0/1） */
    float   position;           /* 线位置，负=左 正=右 0=正中 */
    uint8_t online;            /* 1=检测到线，0=全线丢失 */
} Track_Data;

void        Track_Init(void);
Track_Data  Track_Read(void);
float       Track_GetPosition(void);
int         Track_IsAllBlack(void);   /* 黑路数 >= TRACK_ALLBLACK_MIN（路口）返回 1 */
int         Track_IsThreeBlack(void); /* 三路黑返回 1 */

#ifdef __cplusplus
}
#endif

#endif
