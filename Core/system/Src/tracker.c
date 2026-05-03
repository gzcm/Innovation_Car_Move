#include "tracker.h"

static float base_rps;
static float k_track;
static float pos_filtered = 0.0f;

/*
 * 调参说明：
 *   POS_FILTER_ALPHA_SLOW  直线段滤波系数，调大响应更快但直线更抖，调小更平滑
 *   POS_FILTER_ALPHA_FAST  弯道段滤波系数，调大弯道响应更快，调小更平滑
 *   POS_CORNER_THRESH      弯道触发阈值，调大更晚进入弯道模式，调小更早触发
 *   CORNER_SPEED_RATIO     弯道速度缩减比例，调大弯道速度更快（可能冲出），调小更稳
 *   POS_DEAD_ZONE          死区，调大中心容忍范围更宽，调小对偏差更敏感
 */
#define POS_FILTER_ALPHA_SLOW   0.3f
#define POS_FILTER_ALPHA_FAST   0.7f
#define POS_CORNER_THRESH       0.8f
#define CORNER_SPEED_RATIO      0.55f
#define POS_DEAD_ZONE           0.4f

/* 内联绝对值，避免引入 math.h */
#define FABS(x)  ((x) < 0.0f ? -(x) : (x))

void Tracker_Init(float base, float k)
{
    base_rps     = base;
    k_track      = k;
    pos_filtered = 0.0f;
}

void Tracker_SetSpeed(float base)
{
    base_rps = base;
}

/*
 * 后续提速改进方向：用 IMU gyroZ 替换 position 阈值做弯道检测
 *
 * 当前方案用 pos_filtered > POS_CORNER_THRESH 判断弯道，存在滞后——
 * 车已经偏离才触发降速，速度越高越来不及。
 *
 * 改进方案：为 Tracker_Update 增加 gyroZ 参数（来自 HWT101_GetData()->gyroZ），
 * 用 |gyroZ| > GYRO_CORNER_THRESH（建议初始值 30.0f °/s）判断弯道：
 *   - 车身开始转动时立刻降速，比位置检测超前半个控制周期以上
 *   - |gyroZ| 回落到阈值以下时立刻恢复全速，消除出弯后的多余减速段
 *
 * 接口变更：Tracker_Update(float position, float gyroZ, float *tgt_l, float *tgt_r)
 * 弯道判断：用 FABS(gyroZ) >= GYRO_CORNER_THRESH 替换 FABS(pos_filtered) >= POS_CORNER_THRESH
 * position 仍用于计算差速 diff，gyroZ 仅用于状态切换，不参与 diff 计算
 */

/*
 * 循迹差速计算（自适应弯道策略）
 * position: 线位置，负=偏左 正=偏右
 * 直线段：慢滤波+死区，抑制抖动
 * 弯道段：快滤波+降速，保证转弯跟随能力
 */
void Tracker_Update(float position, float *target_left, float *target_right)
{
    /* 自适应滤波：弯道快速跟随，出弯（回到死区）快速清除残余 */
    float alpha;
    if (FABS(position) >= POS_CORNER_THRESH) {
        alpha = POS_FILTER_ALPHA_FAST;          /* 弯道：快速响应 */
    } else if (FABS(position) < POS_DEAD_ZONE) {
        alpha = 1.0f;                           /* 出弯归零：立即清除滤波器残余 */
    } else {
        alpha = POS_FILTER_ALPHA_SLOW;          /* 直线微偏：慢滤波抑制抖动 */
    }
    pos_filtered = alpha * position + (1.0f - alpha) * pos_filtered;

    /* 死区仅在直线段（小偏差）生效 */
    float pos_used = (FABS(pos_filtered) < POS_DEAD_ZONE) ? 0.0f : pos_filtered;

    /* 弯道降速：速度过快时物理上无法完成小半径转弯 */
    float effective_base = (FABS(pos_filtered) >= POS_CORNER_THRESH)
                           ? base_rps * CORNER_SPEED_RATIO : base_rps;

    float diff = k_track * pos_used;
    *target_left  = effective_base + diff;
    *target_right = effective_base - diff;
}
