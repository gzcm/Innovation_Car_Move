#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

/* 带前馈的速度 PI 控制器 */
typedef struct {
    float kff;       /* 前馈系数 (%/rps) */
    float kp;
    float ki;        /* 积分增益 (%/rps/s) */
    float dt;        /* 采样周期 (s) */
    float integral;
    float output;    /* 上次输出 (-100 ~ 100 %) */
} SpeedPI_t;

void  SpeedPI_Init(SpeedPI_t *pid, float kff, float kp, float ki, float dt);
float SpeedPI_Update(SpeedPI_t *pid, float target_rps, float measured_rps);
void  SpeedPI_Reset(SpeedPI_t *pid);

#ifdef __cplusplus
}
#endif

#endif
