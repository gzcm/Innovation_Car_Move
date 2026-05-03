/**
*  @brief HWT101 单轴(Z轴)晶体陀螺仪倾角传感器驱动
*         通讯串口: 由 HWT101_init() 参数传入, 默认波特率9600
*         传感器上电后自动连续输出数据，无需轮询
*
*  @use   推荐使用 HWT101_Update() 更新数据，然后从 HWT101_GetData() 获取结构体指针读取各项数据
*         也可使用 ReadAngle() / ChangeAngle() 仅获取Yaw角度
*
*  @note  HWT101 仅测量Z轴角速度和Z轴角度(Yaw)，Roll/Pitch数据无意义
*         静态精度: 0.05°, 动态精度: 0.1°
*         输出频率: 0.1Hz ~ 200Hz 可配置
**/
#ifndef HWT101_H
#define HWT101_H

#include "wit_c_sdk.h"
#include "stm32f1xx_hal.h"

/** 传感器数据更新标志位 **/
#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80

/** HWT101 传感器数据结构体 **/
typedef struct {
    float yaw;          /**< Z轴偏航角 (°), 范围 -180 ~ +180 */
    float gyroZ;        /**< Z轴角速度 (°/s), 范围 -2000 ~ +2000 */
} HWT101_Data_t;

typedef struct {
    uint32_t rxBytes;
    uint32_t updateCallbacks;
    uint32_t dataUpdates;
    uint32_t uartErrors;
    uint32_t halErrorCode;
    uint8_t rxStartStatus;
    uint8_t lastUpdateFlags;
} HWT101_Status_t;

/*========== 初始化 ==========*/

/**
 * @brief  初始化HWT101传感器，注册SDK回调并开启串口中断接收
 * @param  huart 串口句柄指针
 * @return 1=成功, 0=串口中断初始化失败
 */
uint8_t HWT101_init(UART_HandleTypeDef *huart);

/**
 * @brief  HWT101 UART接收处理函数，由中央回调分发器调用
 * @param  huart 串口句柄指针
 * @note   此函数应在stm32f1xx_it.c的HAL_UART_RxCpltCallback中调用
 */
void HWT101_UART_RxHandler(UART_HandleTypeDef *huart);
void HWT101_UART_ErrorHandler(UART_HandleTypeDef *huart);

/*========== 数据读取 ==========*/

/**
 * @brief  获取传感器数据结构体指针(只读)
 * @return 指向内部 HWT101_Data_t 的常量指针
 */
const HWT101_Data_t* HWT101_GetData(void);
const HWT101_Status_t* HWT101_GetStatus(void);

/**
 * @brief  检查并更新传感器数据，当有新数据到达时更新内部结构体
 * @return 更新标志位的快照(可用于判断哪些数据更新了)
 */
uint8_t HWT101_Update(void);

/**
 * @brief  获取Z轴角速度
 * @return Z轴角速度(°/s), 范围 -2000 ~ +2000; 未更新返回0.0f
 */
float HWT101_ReadGyroZ(void);

/*========== 校准 ==========*/

/**
 * @brief  Z轴角度归零 (CALIYAW), 立即将当前Yaw角设为0
 * @return WIT_HAL_OK=成功
 */
int32_t HWT101_CalibrateYawZero(void);

/**
 * @brief  陀螺仪+加速度计校准，传感器需静止水平放置
 * @return WIT_HAL_OK=成功
 */
int32_t HWT101_CalibrateGyroAcc(void);
int32_t HWT101_StopGyroAccCalibration(void);

/*========== 配置 ==========*/

/**
 * @brief  设置数据输出频率
 * @param  rate 回传率，使用REG.h中的RRATE_xxx宏 (如 RRATE_10HZ, RRATE_50HZ)
 * @return WIT_HAL_OK=成功
 * @note   设置后需调用 HWT101_SaveConfig() 保存
 */
int32_t HWT101_SetOutputRate(int32_t rate);

/**
 * @brief  设置串口波特率
 * @param  baud 波特率索引，使用REG.h中的WIT_BAUD_xxx宏 (如 WIT_BAUD_9600)
 * @return WIT_HAL_OK=成功
 * @note   设置后需调用 HWT101_SaveConfig() 保存，保存后传感器会以新波特率通讯
 */
int32_t HWT101_SetBaudRate(int32_t baud);

/**
 * @brief  设置滤波带宽
 * @param  bandwidth 带宽，使用REG.h中的BANDWIDTH_xxx宏 (如 BANDWIDTH_21HZ)
 * @return WIT_HAL_OK=成功
 * @note   设置后需调用 HWT101_SaveConfig() 保存
 */
int32_t HWT101_SetBandwidth(int32_t bandwidth);

/**
 * @brief  设置输出数据内容
 * @param  rsw 输出内容位掩码，使用REG.h中的RSW_xxx宏组合 (如 RSW_ACC | RSW_GYRO | RSW_ANGLE)
 * @return WIT_HAL_OK=成功
 * @note   设置后需调用 HWT101_SaveConfig() 保存
 */
int32_t HWT101_SetOutputContent(int32_t rsw);

/**
 * @brief  保存当前配置到传感器Flash
 * @return WIT_HAL_OK=成功
 */
int32_t HWT101_SaveConfig(void);

/**
 * @brief  软件重启传感器
 * @return WIT_HAL_OK=成功
 */
int32_t HWT101_Reboot(void);

#endif
