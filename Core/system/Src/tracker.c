#include "tracker.h"

static float base_rps;
static float k_track;

void Tracker_Init(float base, float k)
{
    base_rps = base;
    k_track  = k;
}

void Tracker_SetSpeed(float base)
{
    base_rps = base;
}

/*
 * 循迹差速计算
 * position: 线位置，负=偏左 正=偏右
 * 输出左右轮目标转速，线偏左时左轮减速右轮加速将车头拉回
 */
void Tracker_Update(float position, float *target_left, float *target_right)
{
    float diff = k_track * position;
    *target_left  = base_rps + diff;
    *target_right = base_rps - diff;
}
