#include "tb6612.h"

/* 内部函数前向声明 */
static int16_t  TB6612_ClampPercent(int16_t speed_percent);
static uint16_t TB6612_PercentToCompare(TB6612_HandleTypeDef *hdev, int16_t speed_percent);
static void     TB6612_WriteDirection(const TB6612_MotorConfig *motor, int16_t speed_percent);
static void     TB6612_ApplyMotor(TB6612_HandleTypeDef *hdev, TB6612_MotorId motor_id, int16_t speed_percent);

/* ================================================================
 * 底层驱动区 —— 直接操作寄存器/GPIO，不对外暴露
 * ================================================================ */

/**
 * @brief  将速度百分比限幅至 [-MAX, +MAX]
 */
static int16_t TB6612_ClampPercent(int16_t speed_percent)
{
    if (speed_percent > TB6612_PWM_MAX_PERCENT)
        return TB6612_PWM_MAX_PERCENT;

    if (speed_percent < -TB6612_PWM_MAX_PERCENT)
        return -TB6612_PWM_MAX_PERCENT;

    return speed_percent;
}

/**
 * @brief  将速度百分比（绝对值）转换为定时器比较寄存器值
 */
static uint16_t TB6612_PercentToCompare(TB6612_HandleTypeDef *hdev, int16_t speed_percent)
{
    uint32_t abs_percent = (speed_percent >= 0) ? (uint32_t)speed_percent
                                                : (uint32_t)(-speed_percent);
    uint32_t period      = hdev->htim->Init.Period;
    return (uint16_t)((abs_percent * period) / TB6612_PWM_MAX_PERCENT);
}

/**
 * @brief  根据速度方向设置 IN1/IN2 GPIO 电平
 * @note   speed_percent > 0 正转，< 0 反转，= 0 停止（双低电平）
 */
static void TB6612_WriteDirection(const TB6612_MotorConfig *motor, int16_t speed_percent)
{
    GPIO_PinState in1_state = GPIO_PIN_RESET;
    GPIO_PinState in2_state = GPIO_PIN_RESET;

    if (speed_percent > 0)
    {
        /* 正转：IN1=1 IN2=0（极性为正时） */
        in1_state = (motor->forward_polarity > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;
        in2_state = (motor->forward_polarity > 0) ? GPIO_PIN_RESET : GPIO_PIN_SET;
    }
    else if (speed_percent < 0)
    {
        /* 反转：IN1=0 IN2=1（极性为正时） */
        in1_state = (motor->forward_polarity > 0) ? GPIO_PIN_RESET : GPIO_PIN_SET;
        in2_state = (motor->forward_polarity > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    }
    /* speed_percent == 0：保持双低电平，电机停止 */

    HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, in1_state);
    HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, in2_state);
}

/**
 * @brief  设置单个电机的方向和 PWM 占空比
 */
static void TB6612_ApplyMotor(TB6612_HandleTypeDef *hdev, TB6612_MotorId motor_id, int16_t speed_percent)
{
    int16_t  limited_speed = TB6612_ClampPercent(speed_percent);
    uint16_t compare_value = TB6612_PercentToCompare(hdev, limited_speed);

    const TB6612_MotorConfig *motor = (motor_id == TB6612_MOTOR_LEFT) ? &hdev->left : &hdev->right;

    TB6612_WriteDirection(motor, limited_speed);
    __HAL_TIM_SET_COMPARE(hdev->htim, motor->pwm_channel, compare_value);
}

/* ================================================================
 * 上层调用区 —— 对外接口，供应用层直接使用
 * ================================================================ */

/**
 * @brief  TB6612 初始化：启动 PWM 输出并停止所有电机
 * @param  hdev  设备句柄指针
 */
void TB6612_Init(TB6612_HandleTypeDef *hdev)
{
    HAL_TIM_PWM_Start(hdev->htim, hdev->left.pwm_channel);
    HAL_TIM_PWM_Start(hdev->htim, hdev->right.pwm_channel);
    TB6612_StopAll(hdev);
}

/**
 * @brief  停止所有电机：PWM 归零，方向引脚全部拉低
 * @param  hdev  设备句柄指针
 */
void TB6612_StopAll(TB6612_HandleTypeDef *hdev)
{
    __HAL_TIM_SET_COMPARE(hdev->htim, hdev->left.pwm_channel,  0U);
    __HAL_TIM_SET_COMPARE(hdev->htim, hdev->right.pwm_channel, 0U);

    HAL_GPIO_WritePin(hdev->left.in1_port,  hdev->left.in1_pin,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(hdev->left.in2_port,  hdev->left.in2_pin,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(hdev->right.in1_port, hdev->right.in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(hdev->right.in2_port, hdev->right.in2_pin, GPIO_PIN_RESET);
}

/**
 * @brief  设置单个电机速度
 * @param  hdev          设备句柄指针
 * @param  motor_id      电机编号（TB6612_MOTOR_LEFT / TB6612_MOTOR_RIGHT）
 * @param  speed_percent 速度百分比，正数前进，负数后退，范围 [-100, 100]
 */
void TB6612_SetMotor(TB6612_HandleTypeDef *hdev, TB6612_MotorId motor_id, int16_t speed_percent)
{
    TB6612_ApplyMotor(hdev, motor_id, speed_percent);
}

/**
 * @brief  同时设置左右电机速度
 * @param  hdev                设备句柄指针
 * @param  left_speed_percent  左电机速度百分比，范围 [-100, 100]
 * @param  right_speed_percent 右电机速度百分比，范围 [-100, 100]
 */
void TB6612_SetMotorPair(TB6612_HandleTypeDef *hdev, int16_t left_speed_percent, int16_t right_speed_percent)
{
    TB6612_ApplyMotor(hdev, TB6612_MOTOR_LEFT,  left_speed_percent);
    TB6612_ApplyMotor(hdev, TB6612_MOTOR_RIGHT, right_speed_percent);
}
