/*
 * car_ctrl.c — 整车运动控制应用层
 *
 * 职责：
 *   - 管理运行模式（停止 / 直线 / 循迹 / 转弯）
 *   - 调度控制环：编码器采样 → 模式分发 → 电机输出
 *   - 对上层暴露简洁接口，对下层通过注册的驱动回调访问硬件
 *
 * 转弯控制架构（Yaw PD 外环 → 轮速 SpeedPI 内环级联）：
 *   外环 PD：angle_err(°) × kp - gyroZ × kd → target_rps，限幅 ±rps_turn_max
 *   内环 PI：左轮目标 -target_rps，右轮目标 +target_rps（原地旋转）
 *            复用行驶 SpeedPI，含前馈+PI+抗积分饱和
 *
 * 控制环执行位置：
 *   update() 应在定时器 ISR 中以固定周期调用（与 sample_time_s 一致）
 */

#include "car_ctrl.h"
#include "pid.h"
#include "tracker.h"

/* ---------- 转弯参数 ---------- */
#define TURN_DONE_DEG   2.0f    /* 角度误差小于此值视为转弯完成 */

/* ---------- 辅助宏 ---------- */
#define FABS(x)  ((x) < 0.0f ? -(x) : (x))

/* ---------- 模块内部状态 ---------- */
static CarCtrl_Driver_t s_drv;
static SpeedPI_t        s_pid_left;
static SpeedPI_t        s_pid_right;
static float            s_kp_angle    = 0.015f;
static float            s_kd_gyro     = 0.003f;
static float            s_rps_max     = 1.0f;
static float            s_turn_fwd    = 0.0f;
static float            s_turn_fwd_active = 0.0f; /* 当前转弯使用的前进基速，180° 掉头时强制为 0 */
static CarMode_t        s_mode        = CAR_MODE_STOP;
static CarCtrl_Diag_t   s_diag        = {0};
static float            s_turn_target = 0.0f;

/* ---------- 内部函数前向声明 ---------- */
static void      s_set_mode(CarMode_t mode);
static void      s_set_speed(float rps);
static void      s_update(void);
static void      s_get_diag(CarCtrl_Diag_t *out);
static void      s_start_turn(int8_t dir);
static void      s_start_turn_180(void);
static CarMode_t s_get_mode(void);

/* ---------- 公开入口 ---------- */

void CarCtrl_Init(CarCtrl_t *ctrl, const CarCtrl_Driver_t *drv, const CarCtrl_Config_t *cfg)
{
    s_drv  = *drv;
    s_mode = CAR_MODE_STOP;
    s_diag = (CarCtrl_Diag_t){0};

    SpeedPI_Init(&s_pid_left,  cfg->kff_left,  cfg->kp, cfg->ki, cfg->sample_time_s);
    SpeedPI_Init(&s_pid_right, cfg->kff_right, cfg->kp, cfg->ki, cfg->sample_time_s);
    Tracker_Init(cfg->base_rps, cfg->k_track);

    s_kp_angle        = cfg->kp_turn_angle;
    s_kd_gyro         = cfg->kd_turn_gyro;
    s_rps_max         = cfg->rps_turn_max;
    s_turn_fwd        = cfg->turn_fwd_rps;
    s_turn_fwd_active = cfg->turn_fwd_rps;

    ctrl->set_mode   = s_set_mode;
    ctrl->set_speed  = s_set_speed;
    ctrl->update     = s_update;
    ctrl->get_diag   = s_get_diag;
    ctrl->start_turn     = s_start_turn;
    ctrl->start_turn_180 = s_start_turn_180;
    ctrl->get_mode       = s_get_mode;
}

/* ---------- 内部实现 ---------- */

static void s_set_mode(CarMode_t mode)
{
    if (mode == CAR_MODE_STOP) {
        SpeedPI_Reset(&s_pid_left);
        SpeedPI_Reset(&s_pid_right);
        s_drv.motor_stop();
        s_diag = (CarCtrl_Diag_t){0};
    }
    s_mode = mode;
}

static void s_set_speed(float rps)
{
    Tracker_SetSpeed(rps);
}

/*
 * 触发路口转弯
 * dir: +1 右转，-1 左转
 * 只在 TRACK / STRAIGHT 模式下有效，避免重入
 */
static void s_start_turn(int8_t dir)
{
    if (s_mode != CAR_MODE_TRACK && s_mode != CAR_MODE_STRAIGHT) return;

    /* 先制动，消除循迹 PWM 冲击 */
    s_drv.motor_stop();

    float cur = s_drv.get_yaw();
    s_turn_target = cur - (float)dir * 90.0f;
    if (s_turn_target >  180.0f) s_turn_target -= 360.0f;
    if (s_turn_target <= -180.0f) s_turn_target += 360.0f;

    s_turn_fwd_active = s_turn_fwd;
    SpeedPI_Reset(&s_pid_left);
    SpeedPI_Reset(&s_pid_right);
    s_mode = CAR_MODE_TURN;
}

/*
 * 触发 180° 掉头
 * 计算当前朝向 +180° 作为目标，最短路径旋转
 * 强制原地旋转（前进基速=0），避免漂移导致越过终点黑线后误触发返程路口
 */
static void s_start_turn_180(void)
{
    if (s_mode != CAR_MODE_TRACK && s_mode != CAR_MODE_STRAIGHT) return;

    s_drv.motor_stop();

    float cur = s_drv.get_yaw();
    s_turn_target = cur + 180.0f;
    if (s_turn_target > 180.0f) s_turn_target -= 360.0f;

    s_turn_fwd_active = 0.0f;
    SpeedPI_Reset(&s_pid_left);
    SpeedPI_Reset(&s_pid_right);
    s_mode = CAR_MODE_TURN;
}

static CarMode_t s_get_mode(void)
{
    return s_mode;
}

/*
 * 控制环主体，在定时器 ISR 中以固定周期调用
 */
static void s_update(void)
{
    s_drv.encoder_update();

    if (s_mode == CAR_MODE_STOP) return;

    /* ---- 转弯模式：Yaw PD 外环 → 轮速 SpeedPI 内环 ---- */
    if (s_mode == CAR_MODE_TURN) {
        float angle_err = s_turn_target - s_drv.get_yaw();
        if (angle_err >  180.0f) angle_err -= 360.0f;
        if (angle_err <= -180.0f) angle_err += 360.0f;

        if (FABS(angle_err) < TURN_DONE_DEG) {
            s_drv.motor_stop();
            SpeedPI_Reset(&s_pid_left);
            SpeedPI_Reset(&s_pid_right);
            s_mode = CAR_MODE_TRACK;
            return;
        }

        /* 外环 PD：角度误差 → 目标轮速 */
        float target_rps = s_kp_angle * angle_err - s_kd_gyro * s_drv.get_gyroZ();
        if (target_rps >  s_rps_max) target_rps =  s_rps_max;
        if (target_rps < -s_rps_max) target_rps = -s_rps_max;

        /* 内环 SpeedPI：差速叠加前进基速 */
        float spd_l, spd_r;
        s_drv.get_speeds(&spd_l, &spd_r);
        float pwm_l = SpeedPI_Update(&s_pid_left,  s_turn_fwd_active - target_rps, spd_l);
        float pwm_r = SpeedPI_Update(&s_pid_right, s_turn_fwd_active + target_rps, spd_r);
        s_drv.set_motor_pwm((int16_t)pwm_l, (int16_t)pwm_r);
        return;
    }

    /* ---- 循迹 / 直线模式：速度 PI 闭环 ---- */
    float spd_l, spd_r;
    s_drv.get_speeds(&spd_l, &spd_r);

    float position = (s_mode == CAR_MODE_TRACK) ? s_drv.get_track_position() : 0.0f;

    float tgt_l, tgt_r;
    Tracker_Update(position, &tgt_l, &tgt_r);

    float pwm_l = SpeedPI_Update(&s_pid_left,  tgt_l, spd_l);
    float pwm_r = SpeedPI_Update(&s_pid_right, tgt_r, spd_r);
    s_drv.set_motor_pwm((int16_t)pwm_l, (int16_t)pwm_r);

    s_diag = (CarCtrl_Diag_t){
        .position    = position,
        .tgt_left    = tgt_l,
        .tgt_right   = tgt_r,
        .speed_left  = spd_l,
        .speed_right = spd_r,
        .pwm_left    = pwm_l,
        .pwm_right   = pwm_r,
    };
}

static void s_get_diag(CarCtrl_Diag_t *out)
{
    *out = s_diag;
    out->turn_target = s_turn_target;  /* 始终输出最新目标，便于调试 */
}
