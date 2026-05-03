#include "app.h"
#include "main.h"
#include "tim.h"
#include "i2c.h"
#include "usart.h"
#include "OLED.h"
#include "tb6612.h"
#include "button.h"
#include "encoder.h"
#include "vofa.h"
#include "track.h"
#include "HWT101.h"
#include "car_ctrl.h"
#include <stdio.h>

/* ---------- 硬件参数 ---------- */
#define ENCODER_SAMPLE_TIME_S           (0.02f)
#define ENCODER_PULSES_PER_REVOLUTION   (2040.0f)
#define ENCODER_LEFT_POLARITY           (1)
#define ENCODER_RIGHT_POLARITY          (1)
#define ENCODER_SPEED_FILTER_ALPHA      (0.3f)

/* 速度 PI 初始参数（根据开环测量整定，后续按实测调整） */
#define PID_KFF_LEFT    (24.8f)   /* 1 / 0.0403 rps/% */
#define PID_KFF_RIGHT   (22.8f)   /* 1 / 0.0438 rps/% */
#define PID_KP          (25.0f)
#define PID_KI          (120.0f)  /* Kp/Ti，Ti ≈ 0.2s */

#define TRACKER_K_TRACK  (0.6f)

/*
 * 转弯级联参数（Yaw PD 外环 → 目标轮速 → SpeedPI 内环）
 *   TURN_KP_ANGLE：外环 P 增益（rps/°）
 *     err=90° × 0.015 = 1.35 rps → 被 TURN_RPS_MAX 限幅
 *     增大 → 响应快但易超调；建议从 0.010 开始观察
 *   TURN_KD_GYRO：外环 D 阻尼（rps/(°/s)）
 *     gyroZ=100°/s 时阻尼 = 0.003×100 = 0.3 rps
 *     仍超调则增大；运动迟钝则减小
 *   TURN_RPS_MAX：外环输出上限（rps），建议不超过正常循迹速度
 */
#define TURN_KP_ANGLE   (0.015f)  /* rps/°  */
#define TURN_KD_GYRO    (0.003f)  /* rps/(°/s) */
#define TURN_RPS_MAX    (1.0f)    /* 差速限幅（rps） */
#define TURN_FWD_RPS    (0.25f)    /* 转弯前进基速（rps），0 = 原地旋转 */

/* 目标转速档位（rps），按键逐档切换 */
static const float RPS_STEPS[] = { 0.0f, 0.5f, 1.0f, 1.5f, 2.0f, 2.5f };
#define RPS_STEPS_LEN  (sizeof(RPS_STEPS) / sizeof(RPS_STEPS[0]))

/* ---------- 硬件句柄 ---------- */
static TB6612_HandleTypeDef hTB6612 = {
    .htim = &htim3,
    .left = {
        .in1_port        = MOTOR_L_IN1_GPIO_Port,
        .in1_pin         = MOTOR_L_IN1_Pin,
        .in2_port        = MOTOR_L_IN2_GPIO_Port,
        .in2_pin         = MOTOR_L_IN2_Pin,
        .pwm_channel     = TIM_CHANNEL_2,
        .forward_polarity = 1,
    },
    .right = {
        .in1_port        = MOTOR_R_IN1_GPIO_Port,
        .in1_pin         = MOTOR_R_IN1_Pin,
        .in2_port        = MOTOR_R_IN2_GPIO_Port,
        .in2_pin         = MOTOR_R_IN2_Pin,
        .pwm_channel     = TIM_CHANNEL_1,
        .forward_polarity = 1,
    },
};

/* 编码器句柄：TIM2 左轮，TIM4 右轮 */
static Encoder_HandleTypeDef hEncoder = {
    .left_htim      = &htim2,
    .right_htim     = &htim4,
    .left_polarity  = ENCODER_LEFT_POLARITY,
    .right_polarity = ENCODER_RIGHT_POLARITY,
    .sample_time_s  = ENCODER_SAMPLE_TIME_S,
    .pulses_per_rev = ENCODER_PULSES_PER_REVOLUTION,
    .filter_alpha   = ENCODER_SPEED_FILTER_ALPHA,
};

/* ---------- 应用状态 ---------- */
static CarCtrl_t        s_car_ctrl;
static volatile float   target_rps  = 0.0f;
static volatile uint8_t vofa_pending = 0;

/* ---------- CarCtrl 驱动回调 ---------- */
static void drv_encoder_update(void)
{
    Encoder_Update(&hEncoder);
}

static void drv_get_speeds(float *left, float *right)
{
    Encoder_SpeedSample s = Encoder_GetSpeedSample(&hEncoder);
    *left  = s.left_speed_rps;
    *right = s.right_speed_rps;
}

static void drv_set_motor_pwm(int16_t left, int16_t right)
{
    TB6612_SetMotorPair(&hTB6612, left, right);
}

static void drv_motor_stop(void)
{
    TB6612_StopAll(&hTB6612);
}

static float drv_get_yaw(void)
{
    return HWT101_GetData()->yaw;
}

static float drv_get_gyroZ(void)
{
    return HWT101_GetData()->gyroZ;
}

static const CarCtrl_Driver_t s_car_drv = {
    .encoder_update     = drv_encoder_update,
    .get_speeds         = drv_get_speeds,
    .set_motor_pwm      = drv_set_motor_pwm,
    .motor_stop         = drv_motor_stop,
    .get_track_position = Track_GetPosition,
    .get_yaw            = drv_get_yaw,
    .get_gyroZ          = drv_get_gyroZ,
};

static const CarCtrl_Config_t s_car_cfg = {
    .kff_left      = PID_KFF_LEFT,
    .kff_right     = PID_KFF_RIGHT,
    .kp            = PID_KP,
    .ki            = PID_KI,
    .sample_time_s = ENCODER_SAMPLE_TIME_S,
    .base_rps      = 0.5f,
    .k_track       = TRACKER_K_TRACK,
    .kp_turn_angle = TURN_KP_ANGLE,
    .kd_turn_gyro  = TURN_KD_GYRO,
    .rps_turn_max  = TURN_RPS_MAX,
    .turn_fwd_rps  = TURN_FWD_RPS,
};

/* ---------- 公开接口 ---------- */

void App_Init(void)
{
    OLED_Init(&hi2c2);
    TB6612_Init(&hTB6612);
    Encoder_Init(&hEncoder);
    CarCtrl_Init(&s_car_ctrl, &s_car_drv, &s_car_cfg);
    HWT101_init(&huart2);
    HAL_TIM_Base_Start_IT(&htim1);

    OLED_ShowString(1, 1, "Speed PID Ctrl  ");
    OLED_ShowString(2, 1, "Press btn start ");
    OLED_ShowString(3, 1, "                ");
    OLED_ShowString(4, 1, "                ");
}

void App_Update(void)
{
    HWT101_Update();

    /* 按钮逐档切换目标转速：0 → 0.5 → 1.0 → 1.5 → 2.0 → 2.5 → 0 → ... */
    if (Button_CheckToggleRequest())
    {
        static uint8_t step = 0;
        step = (step + 1) % RPS_STEPS_LEN;
        target_rps = RPS_STEPS[step];
        s_car_ctrl.set_speed(target_rps);
        s_car_ctrl.set_mode(target_rps == 0.0f ? CAR_MODE_STOP : CAR_MODE_TRACK);
    }

    /* 路口检测：四路全黑时触发 90° 右转
     * cross_armed 防重入：触发后锁定，转弯完成（mode 离开 TURN）后立即解锁
     * 不依赖传感器离开全黑区，高速连续路口也能正常响应 */
    {
        static uint8_t cross_armed  = 1;
        static uint8_t was_turning  = 0;
        int all_black = Track_IsAllBlack();
        CarMode_t mode = s_car_ctrl.get_mode();

        /* 转弯刚完成（TURN → 其他）时立即解锁 */
        if (was_turning && mode != CAR_MODE_TURN) {
            cross_armed = 1;
        }
        was_turning = (mode == CAR_MODE_TURN);

        if (all_black && cross_armed) {
            s_car_ctrl.start_turn(+1);
            cross_armed = 0;
        }
    }

    /* VOFA 数据上发（由 TIM1 ISR 触发，约 50Hz） */
    if (vofa_pending)
    {
        vofa_pending = 0;
        CarCtrl_Diag_t diag;
        s_car_ctrl.get_diag(&diag);
        float vofa_data[8] = {
            target_rps,
            diag.position,
            diag.tgt_left,
            diag.tgt_right,
            diag.speed_left,
            diag.speed_right,
            diag.pwm_left,
            diag.pwm_right,
        };
        VOFA_Send(vofa_data, 8);
    }

    /* OLED 刷新（约 5Hz） */
    {
        static uint32_t last_tick = 0;
        static char disp_buf[17];
        if (HAL_GetTick() - last_tick >= 200)
        {
            last_tick = HAL_GetTick();
            const HWT101_Data_t *imu = HWT101_GetData();

            snprintf(disp_buf, sizeof(disp_buf), "Tgt:%+6.2f rps  ", (double)target_rps);
            OLED_ShowString(1, 1, disp_buf);

            snprintf(disp_buf, sizeof(disp_buf), "Yaw:%+7.1f deg  ", (double)imu->yaw);
            OLED_ShowString(2, 1, disp_buf);

            const char *mode_str;
            switch (s_car_ctrl.get_mode()) {
                case CAR_MODE_STOP:     mode_str = "Mode: STOP      "; break;
                case CAR_MODE_STRAIGHT: mode_str = "Mode: STRAIGHT  "; break;
                case CAR_MODE_TRACK:    mode_str = "Mode: TRACK     "; break;
                case CAR_MODE_TURN:     mode_str = "Mode: TURNING   "; break;
                default:                mode_str = "Mode: ???       "; break;
            }
            OLED_ShowString(3, 1, mode_str);
        }
    }
}

/* TIM1 周期中断体，每 20ms 执行一次控制环 */
void App_TimerISR(void)
{
    s_car_ctrl.update();
    if (target_rps != 0.0f) vofa_pending = 1;
}
