#include "pid.h"

void SpeedPI_Init(SpeedPI_t *pid, float kff, float kp, float ki, float dt)
{
    pid->kff      = kff;
    pid->kp       = kp;
    pid->ki       = ki;
    pid->dt       = dt;
    pid->integral = 0.0f;
    pid->output   = 0.0f;
}

float SpeedPI_Update(SpeedPI_t *pid, float target_rps, float measured_rps)
{
    float err = target_rps - measured_rps;
    float ff  = pid->kff * target_rps;

    pid->integral += pid->ki * err * pid->dt;

    float out = ff + pid->kp * err + pid->integral;

    /* 限幅 + 抗积分饱和：输出饱和时回退本步积分 */
    if (out > 100.0f) {
        pid->integral -= pid->ki * err * pid->dt;
        out = 100.0f;
    } else if (out < -100.0f) {
        pid->integral -= pid->ki * err * pid->dt;
        out = -100.0f;
    }

    pid->output = out;
    return out;
}

void SpeedPI_Reset(SpeedPI_t *pid)
{
    pid->integral = 0.0f;
    pid->output   = 0.0f;
}
