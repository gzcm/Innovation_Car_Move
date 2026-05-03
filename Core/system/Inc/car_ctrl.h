#ifndef __CAR_CTRL_H
#define __CAR_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 硬件驱动回调，由调用方注册，car_ctrl 不依赖任何具体 BSP 头文件 */
typedef struct {
    void    (*encoder_update)(void);
    void    (*get_speeds)(float *left, float *right);
    void    (*set_motor_pwm)(int16_t left, int16_t right);
    void    (*motor_stop)(void);
    float   (*get_track_position)(void);
    float   (*get_yaw)(void);          /* IMU Yaw 角（°） */
    float   (*get_gyroZ)(void);        /* IMU Z 轴角速度（°/s），转弯 D 阻尼项 */
} CarCtrl_Driver_t;

/* PID 与循迹初始参数，由调用方传入 */
typedef struct {
    /* 行驶速度 PI */
    float kff_left;
    float kff_right;
    float kp;
    float ki;
    float sample_time_s;
    float base_rps;
    float k_track;
    /* 转弯级联：Yaw PD 外环 → 差速 → SpeedPI 内环，叠加前进基速 */
    float kp_turn_angle;   /* 外环 P 增益（rps/°） */
    float kd_turn_gyro;    /* 外环 D 阻尼（rps/(°/s)），抑制过冲 */
    float rps_turn_max;    /* 差速分量限幅（rps） */
    float turn_fwd_rps;    /* 转弯时前进基速（rps），0 = 原地旋转 */
} CarCtrl_Config_t;

typedef enum {
    CAR_MODE_STOP     = 0,
    CAR_MODE_STRAIGHT = 1,
    CAR_MODE_TRACK    = 2,
    CAR_MODE_TURN     = 3,  /* 路口定角旋转，完成后自动切回 TRACK */
} CarMode_t;

/* 供 VOFA / OLED 读取的诊断数据（上一控制周期快照） */
typedef struct {
    float position;
    float tgt_left;
    float tgt_right;
    float speed_left;
    float speed_right;
    float pwm_left;
    float pwm_right;
} CarCtrl_Diag_t;

/* 整车控制对象，Init 后通过此结构体调用所有接口 */
typedef struct {
    void       (*set_mode)(CarMode_t mode);
    void       (*set_speed)(float rps);
    void       (*update)(void);                 /* 在定时器 ISR 中调用 */
    void       (*get_diag)(CarCtrl_Diag_t *out);
    void       (*start_turn)(int8_t dir);       /* 触发路口转弯：+1 右转，-1 左转 */
    CarMode_t  (*get_mode)(void);               /* 读取当前运行模式 */
} CarCtrl_t;

void CarCtrl_Init(CarCtrl_t *ctrl, const CarCtrl_Driver_t *drv, const CarCtrl_Config_t *cfg);

#ifdef __cplusplus
}
#endif

#endif
