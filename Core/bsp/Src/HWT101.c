#include "HWT101.h"

#define HWT101_AUTO_GYRO_CALIBRATION      1
#define HWT101_GYRO_CALIBRATION_TIME_MS   3000U

/** 串口句柄 **/
static UART_HandleTypeDef *s_huart = NULL;

/** 传感器数据更新标志位, static volatile防止竞态和优化问题 **/
static volatile uint8_t DataUpdate = 0;

/** 串口中断接收临时变量 **/
static uint8_t rx_temp;

/** 传感器数据结构体 **/
static HWT101_Data_t sensorData = {0};
static HWT101_Status_t sensorStatus = {0};

static uint8_t HWT101_RangeContains(uint32_t startReg, uint32_t regNum, uint32_t targetReg)
{
    return (targetReg >= startReg) && (targetReg < (startReg + regNum));
}

/*==========================================================
 *  SDK 回调函数
 *==========================================================*/

/**
 * @brief 串口发送函数，SDK通过此函数向传感器发送指令
 */
static void SensorUartSend(uint8_t *p_ucData, uint32_t uiLen)
{
    HAL_UART_Transmit(s_huart, p_ucData, uiLen, 100);
}

/**
 * @brief 传感器数据更新回调，由SDK在解析到新数据包时调用
 * @note  HWT101仅Z轴有效，温度数据在加速度包(0x51)的第4个字段中
 */
static void SensorDataUpdate(uint32_t uiReg, uint32_t uiRegNum)
{
    sensorStatus.updateCallbacks++;

    if (HWT101_RangeContains(uiReg, uiRegNum, AZ))
    {
        DataUpdate |= ACC_UPDATE;
    }

    if (HWT101_RangeContains(uiReg, uiRegNum, GZ))
    {
        DataUpdate |= GYRO_UPDATE;
    }

    if (HWT101_RangeContains(uiReg, uiRegNum, HZ))
    {
        DataUpdate |= MAG_UPDATE;
    }

    if (HWT101_RangeContains(uiReg, uiRegNum, Yaw))
    {
        DataUpdate |= ANGLE_UPDATE;
    }

    if ((DataUpdate & (ACC_UPDATE | GYRO_UPDATE | MAG_UPDATE | ANGLE_UPDATE)) == 0U)
    {
        DataUpdate |= READ_UPDATE;
    }
}

static void HWT101_DelayMs(uint16_t ucMs)
{
    HAL_Delay(ucMs);
}

/*==========================================================
 *  串口中断回调
 *==========================================================*/

/**
 * @brief HWT101 UART接收处理函数，由中央回调分发器调用
 * @note  不直接定义HAL_UART_RxCpltCallback以避免冲突
 */
void HWT101_UART_RxHandler(UART_HandleTypeDef *huart)
{
    if (s_huart != NULL && huart->Instance == s_huart->Instance)
    {
        HAL_StatusTypeDef status;

        sensorStatus.rxBytes++;
        WitSerialDataIn(rx_temp);
        status = HAL_UART_Receive_IT(s_huart, &rx_temp, 1);
        sensorStatus.rxStartStatus = (uint8_t)status;
        sensorStatus.halErrorCode = s_huart->ErrorCode;
    }
}

void HWT101_UART_ErrorHandler(UART_HandleTypeDef *huart)
{
    if (s_huart != NULL && huart->Instance == s_huart->Instance)
    {
        HAL_StatusTypeDef status;

        sensorStatus.uartErrors++;
        sensorStatus.halErrorCode = s_huart->ErrorCode;
        status = HAL_UART_Receive_IT(s_huart, &rx_temp, 1);
        sensorStatus.rxStartStatus = (uint8_t)status;
    }
}

/*==========================================================
 *  初始化
 *==========================================================*/

uint8_t HWT101_init(UART_HandleTypeDef *huart)
{
    HAL_StatusTypeDef status;

    s_huart = huart;
    WitInit(WIT_PROTOCOL_NORMAL, 0x50);
    WitSerialWriteRegister(SensorUartSend);
    WitRegisterCallBack(SensorDataUpdate);
    WitDelayMsRegister(HWT101_DelayMs);
    status = HAL_UART_Receive_IT(s_huart, &rx_temp, 1);
    sensorStatus.rxStartStatus = (uint8_t)status;
    sensorStatus.halErrorCode = s_huart->ErrorCode;
    if (status != HAL_OK)
    {
        return 0;
    }

#if HWT101_AUTO_GYRO_CALIBRATION
    HWT101_CalibrateGyroAcc();
    HWT101_DelayMs(HWT101_GYRO_CALIBRATION_TIME_MS);
    HWT101_StopGyroAccCalibration();
    HWT101_CalibrateYawZero();
#endif

    return 1;
}

/*==========================================================
 *  数据读取
 *==========================================================*/

const HWT101_Data_t* HWT101_GetData(void)
{
    return &sensorData;
}

const HWT101_Status_t* HWT101_GetStatus(void)
{
    return &sensorStatus;
}

/**
 * @brief 检查数据更新标志并解算传感器数据
 * @return 更新标志位快照
 */
uint8_t HWT101_Update(void)
{
    __disable_irq();
    uint8_t snapshot = DataUpdate;
    DataUpdate = 0;
    __enable_irq();

    if (!snapshot)
        return 0;

    sensorStatus.lastUpdateFlags = snapshot;
    sensorStatus.dataUpdates++;

    if (snapshot & ANGLE_UPDATE)
    {
        sensorData.yaw = sReg[Yaw] / 32768.0f * 180.0f;
    }

    if (snapshot & GYRO_UPDATE)
    {
        sensorData.gyroZ = sReg[GZ] / 32768.0f * 2000.0f;
    }

    return (uint8_t)snapshot;
}

/**
 * @brief 返回Z轴角速度，返回上一次 HWT101_Update() 解算的值
 * @note  需先调用 HWT101_Update()，否则返回的是上次更新的旧值
 */
float HWT101_ReadGyroZ(void)
{
    return sensorData.gyroZ;
}

/*==========================================================
 *  校准
 *==========================================================*/

int32_t HWT101_CalibrateYawZero(void)
{
    return WitStartIYAWCali();
}

int32_t HWT101_CalibrateGyroAcc(void)
{
    return WitStartAccCali();
}

int32_t HWT101_StopGyroAccCalibration(void)
{
    return WitStopAccCali();
}

/*==========================================================
 *  配置
 *==========================================================*/

int32_t HWT101_SetOutputRate(int32_t rate)
{
    return WitSetOutputRate(rate);
}

int32_t HWT101_SetBaudRate(int32_t baud)
{
    return WitSetUartBaud(baud);
}

int32_t HWT101_SetBandwidth(int32_t bandwidth)
{
    return WitSetBandwidth(bandwidth);
}

int32_t HWT101_SetOutputContent(int32_t rsw)
{
    return WitSetContent(rsw);
}

int32_t HWT101_SaveConfig(void)
{
    int32_t ret = WitWriteReg(KEY, KEY_UNLOCK);
    if (ret != WIT_HAL_OK) return ret;
    HWT101_DelayMs(1);
    return WitWriteReg(SAVE, SAVE_PARAM);
}

int32_t HWT101_Reboot(void)
{
    int32_t ret = WitWriteReg(KEY, KEY_UNLOCK);
    if (ret != WIT_HAL_OK) return ret;
    HWT101_DelayMs(1);
    return WitWriteReg(SAVE, SAVE_SWRST);
}
