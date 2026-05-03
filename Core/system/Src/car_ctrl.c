/*
 * car_ctrl.c — 整车运动控制应用层
 *
 * 职责：
 *   - 管理运行模式（停止 / 直线 / 循迹 / 转弯）
 *   - 调度控制环：编码器采样 → 模式分发 → 电机输出
 *   - 对上层暴露简洁接口，对下层通过注册的驱动回调访问硬件
 *
 * 转弯控制架构（单环直接 P）：
 *   angle_err(°) × kp_turn_angle → pwm（%），限幅 ±rps_turn_max（即最大 PWM%）
 *   电机输出：左轮 +pwm，右轮 -pwm（原地旋转）
 *   不经过 SpeedPI，避免积分在小目标下的振荡发散
 *
 * 控制环执行位置：
 *   update() 应在定时器 ISR 中以固定周期调用（与 sample_time_s 一致）
 */

#include "car_ctrl.h"
#include "pid.h"
#include "tracker.h"

/* ---------- 转弯参数 ---------- */
#define TURN_DONE_DEG   5.0f    /* 角度误差小于此值视为转弯完成 */

/* ---------- 辅助宏 ---------- */
#define FABS(x)  ((x) < 0.0f ? -(x) : (x))

/* ---------- 模块内部状态 ---------- */
static CarCtrl_Driver_t s_drv;
static SpeedPI_t        s_pid_left;
static SpeedPI_t        s_pid_right;
static float            s_kp_angle    = 1.0f;
static float            s_kd_gyro     = 0.2f;
static float            s_rps_max     = 40.0f;
static CarMode_t        s_mode        = CAR_MODE_STOP;
static CarCtrl_Diag_t   s_diag        = {0};
static float            s_turn_target = 0.0f;

/* ---------- 内部函数前向声明 ---------- */
static void      s_set_mode(CarMode_t mode);
static void      s_set_speed(float rps);
static void      s_update(void);
static void      s_get_diag(CarCtrl_Diag_t *out);
static void      s_start_turn(int8_t dir);
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

    s_kp_angle = cfg->kp_turn_angle;
    s_kd_gyro  = cfg->kd_turn_gyro;
    s_rps_max  = cfg->rps_turn_max;

    ctrl->set_mode   = s_set_mode;
    ctrl->set_speed  = s_set_speed;
    ctrl->update     = s_update;
    ctrl->get_diag   = s_get_diag;
    ctrl->start_turn = s_start_turn;
    ctrl->get_mode   = s_get_mode;
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

    /* ---- 转弯模式：单环直接 P，angle_err(°) → PWM(%) ---- */
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

        /* PD：P 驱动到位，D 阻尼防过冲 */
        float pwm = s_kp_angle * angle_err - s_kd_gyro * s_drv.get_gyroZ();
        if (pwm >  s_rps_max) pwm =  s_rps_max;
        if (pwm < -s_rps_max) pwm = -s_rps_max;
        s_drv.set_motor_pwm(-(int16_t)pwm, (int16_t)pwm);
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
}
