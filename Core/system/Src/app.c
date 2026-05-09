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
#include "esp8266_app.h"
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

/* ESP8266 指令触发时的起步速度 */
#define ESP8266_START_RPS  (0.75f)

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
    ESP8266_App_Init(&huart5);
    HAL_TIM_Base_Start_IT(&htim1);

    OLED_ShowString(1, 1, "Speed PID Ctrl  ");
    OLED_ShowString(2, 1, "Press btn start ");
    OLED_ShowString(3, 1, "                ");
    OLED_ShowString(4, 1, "                ");
}

void App_Update(void)
{
    HWT101_Update();
    ESP8266_App_Run();

    /* 路口状态（函数级，各子块均可访问） */
    static int8_t  s_turn_dirs[2]    = {0, 0};
    static int8_t  s_return_dirs[2]  = {0, 0};
    static uint8_t s_cross_index     = 0;
    static uint8_t cross_armed       = 1;
    static uint8_t was_turning       = 0;
    static uint8_t cross_clear       = 1;
    static uint8_t s_esp_started     = 0;  /* ESP8266 启动锁，终点后重置允许重新接收指令 */
    static uint8_t s_returning       = 0;  /* 1 = 已掉头进入返程 */
    static uint8_t s_return_pending  = 0;  /* 1 = 已触发 180° 掉头，等待完成 */

    /* ESP8266 收到指令后自动启动循迹 */
    {
        ESP8266_CommandState cmd;
        if (!s_esp_started && ESP8266_App_PopCommand(&cmd))
        {
            ESP8266_App_DecodeDirs(cmd, &s_turn_dirs[0], &s_turn_dirs[1]);
            s_cross_index    = 0;
            cross_armed      = 1;
            was_turning      = 0;
            cross_clear      = 1;
            s_returning      = 0;
            s_return_pending = 0;
            target_rps = ESP8266_START_RPS;
            s_car_ctrl.set_speed(target_rps);
            s_car_ctrl.set_mode(CAR_MODE_TRACK);
            s_esp_started = 1;
        }
    }

    /* 按钮手动调速（兼容调试模式） */
    if (Button_CheckToggleRequest())
    {
        static uint8_t step = 0;
        step = (step + 1) % RPS_STEPS_LEN;
        target_rps = RPS_STEPS[step];
        if (step == 1) {
            /* 从停止启动，设置默认路线 A1（第一路口左转，第二路口右转） */
            s_turn_dirs[0]   = -1;
            s_turn_dirs[1]   = +1;
            s_cross_index    = 0;
            cross_armed      = 1;
            was_turning      = 0;
            cross_clear      = 1;
            s_returning      = 0;
            s_return_pending = 0;
        }
        s_car_ctrl.set_speed(target_rps);
        s_car_ctrl.set_mode(target_rps == 0.0f ? CAR_MODE_STOP : CAR_MODE_TRACK);
    }

    /* 路口 + 终点检测 */
    {
        int all_black   = Track_IsAllBlack();
        int three_black = Track_IsThreeBlack();
        CarMode_t mode  = s_car_ctrl.get_mode();

        if (was_turning && mode != CAR_MODE_TURN) {
            cross_armed = 1;
            cross_clear = 0;  /* 等待离开当前路口区域 */

            /* 180° 掉头完成 → 切换到返程模式 */
            if (s_return_pending) {
                s_return_pending = 0;
                s_returning      = 1;
                s_cross_index    = 0;
                /* 镜像前进方向：返程路口顺序与方向均与去程相反 */
                s_return_dirs[0] = -s_turn_dirs[1];
                s_return_dirs[1] = -s_turn_dirs[0];
            }
        }
        was_turning = (mode == CAR_MODE_TURN);

        /* 离开全黑区域即可重新上锁；3黑也算未离开，避免返程二号路口提前误判清零 */
        if (!all_black && !three_black) cross_clear = 1;

        if (!s_returning) {
            /* 去程：两个路口 + 终点均以全黑触发 */
            if (all_black && cross_armed && cross_clear) {
                if (s_cross_index < 2) {
                    int8_t dir = s_turn_dirs[s_cross_index];
                    s_cross_index++;
                    cross_armed = 0;
                    if (dir != 0) {
                        s_car_ctrl.start_turn(dir);
                    } else {
                        /* 直行过路口：was_turning 不会触发，手动重新上锁 */
                        cross_armed = 1;
                        cross_clear = 0;
                    }
                } else {
                    /* 终点：触发 180° 掉头，进入返程 */
                    s_cross_index    = 0;
                    cross_armed      = 0;
                    s_return_pending = 1;
                    s_car_ctrl.start_turn_180();
                }
            }
        } else {
            /* 返程：一号路口全黑、二号路口 3 黑（含全黑兜底）、出发点全黑停车 */
            int trigger;
            if (s_cross_index == 1) {
                trigger = three_black || all_black;
            } else {
                trigger = all_black;
            }

            if (trigger && cross_armed && cross_clear) {
                if (s_cross_index < 2) {
                    int8_t dir = s_return_dirs[s_cross_index];
                    s_cross_index++;
                    cross_armed = 0;
                    if (dir != 0) {
                        s_car_ctrl.start_turn(dir);
                    } else {
                        cross_armed = 1;
                        cross_clear = 0;
                    }
                } else {
                    /* 抵达出发点，停车并解锁 ESP8266 */
                    s_car_ctrl.set_mode(CAR_MODE_STOP);
                    cross_armed   = 0;
                    s_returning   = 0;
                    s_esp_started = 0;
                }
            }
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

            /* 调试行：模式 + 路口序号 + 上次转弯目标角度
             * 格式：M<mode> i:<idx> t:<target>
             *   mode: 0=STOP 1=STR 2=TRK 3=TURN
             *   idx : 当前 s_cross_index（去程 0/1/2，返程同样从 0 起）
             *   t   : start_turn / start_turn_180 设的目标 yaw（°）
             */
            CarCtrl_Diag_t diag_dbg;
            s_car_ctrl.get_diag(&diag_dbg);
            snprintf(disp_buf, sizeof(disp_buf), "M%d i:%d t:%+04d   ",
                     (int)s_car_ctrl.get_mode(),
                     (int)s_cross_index,
                     (int)diag_dbg.turn_target);
            OLED_ShowString(3, 1, disp_buf);

            snprintf(disp_buf, sizeof(disp_buf), "ESP:%s%s         ",
                     ESP8266_App_IsReady() ? "" : "init ",
                     ESP8266_App_GetStateName());
            OLED_ShowString(4, 1, disp_buf);
        }
    }
}

/* TIM1 周期中断体，每 20ms 执行一次控制环 */
void App_TimerISR(void)
{
    s_car_ctrl.update();
    if (target_rps != 0.0f) vofa_pending = 1;
}
