#include "track.h"

/* 单个传感器 GPIO 描述 */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} TrackPin;

/* 4路传感器物理排列：左→右，硬件顺序按实际 PCB 调整 */
static const TrackPin tracks[TRACK_COUNT] = {
    {Track1_GPIO_Port, Track1_Pin},  /* 最左 */
    {Track2_GPIO_Port, Track2_Pin},  /* 左中 */
    {Track3_GPIO_Port, Track3_Pin},  /* 右中 */
    {Track4_GPIO_Port, Track4_Pin},  /* 最右 */
};

void Track_Init(void)
{
    /* GPIO 已在 MX_GPIO_Init 中初始化 */
}

/*
 * 扫描4路传感器的电平状态
 * TRACK_BLACK_LEVEL 决定黑线对应的高低电平（0 或 1，见 track.h）
 */
Track_Data Track_Read(void)
{
    Track_Data data = {0};

    /* 读取每路 GPIO 电平，转换为 0(白)/1(黑) */
    for (int i = 0; i < TRACK_COUNT; i++) {
        uint8_t level = (HAL_GPIO_ReadPin(tracks[i].port, tracks[i].pin) == GPIO_PIN_SET) ? 1 : 0;
        data.raw[i] = (level == TRACK_BLACK_LEVEL) ? 1 : 0;
    }

    /* 判断是否至少有一路检测到线 */
    for (int i = 0; i < TRACK_COUNT; i++) {
        if (data.raw[i]) {
            data.online = 1;
            break;
        }
    }

    /* 有测线时计算加权位置 */
    if (data.online) {
        int sum_w = 0, sum_n = 0;
        /* 4路权重：-3(最左), -1, 1, 3(最右) → position 范围约 ±3 */
        static const int8_t w[TRACK_COUNT] = {-3, -1, 1, 3};
        for (int i = 0; i < TRACK_COUNT; i++) {
            if (data.raw[i]) {
                sum_w += w[i];
                sum_n += 1;
            }
        }
        data.position = (float)sum_w / (float)sum_n;
    }

    return data;
}

float Track_GetPosition(void)
{
    return Track_Read().position;
}
