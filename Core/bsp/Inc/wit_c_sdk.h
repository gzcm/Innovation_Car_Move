#ifndef __WIT_C_SDK_H
#define __WIT_C_SDK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "REG.h"

//定义程序工作状态
#define WIT_HAL_OK      (0)     /**< There is no error */
#define WIT_HAL_BUSY    (-1)    /**< Busy */
#define WIT_HAL_TIMEOUT (-2)    /**< Timed out */
#define WIT_HAL_ERROR   (-3)    /**< A generic error happens */
#define WIT_HAL_NOMEM   (-4)    /**< No memory */
#define WIT_HAL_EMPTY   (-5)    /**< The resource is empty */
#define WIT_HAL_INVAL   (-6)    /**< Invalid argument */

//缓存区大小
#define WIT_DATA_BUFF_SIZE  256

//定义通讯方式
#define WIT_PROTOCOL_NORMAL 0
#define WIT_PROTOCOL_MODBUS 1
#define WIT_PROTOCOL_CAN    2
#define WIT_PROTOCOL_I2C    3

//串口函数定义
/* serial function */
typedef void (*SerialWrite)(uint8_t *p_ucData, uint32_t uiLen);     /** 函数指针：串口发送函数的抽象 **/
int32_t WitSerialWriteRegister(SerialWrite write_func);             /** SDK注册函数，用于注册串口发送函数 **/
void WitSerialDataIn(uint8_t ucData);                               /** SDK数据输入接口 **/

//IIC函数定义
/* iic function */

/*
    i2c write function example

    int32_t WitI2cWrite(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen)
    {
        i2c_start();
        i2c_send(ucAddr);
        if(i2c_wait_ask() != SUCCESS)return 0;
        i2c_send(ucReg);
        if(i2c_wait_ask() != SUCCESS)return 0;
        for(uint32_t i = 0; i < uiLen; i++)
        {
            i2c_send(*p_ucVal++); 
            if(i2c_wait_ask() != SUCCESS)return 0;
        }
        i2c_stop();
        return 1;
    }
*/
typedef int32_t (*WitI2cWrite)(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen);    /** 函数指针：IIC发送函数的抽象 **/
/*
    i2c read function example

    int32_t WitI2cRead(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen)
    {
        i2c_start();
        i2c_send(ucAddr);
        if(i2c_wait_ask() != SUCCESS)return 0;
        i2c_send(ucReg);
        if(i2c_wait_ask() != SUCCESS)return 0;
        
        i2c_start();
        i2c_send(ucAddr+1);
        for(uint32_t i = 0; i < uiLen; i++)
        {
            if(i+1 == uiLen)*p_ucVal++ = i2c_read(0);  //last byte no ask
            else *p_ucVal++ = i2c_read(1);  //  ask
        }
        i2c_stop();
        return 1;
    }
*/
/** 函数指针：IIC接收函数的抽象 **/
typedef int32_t (*WitI2cRead)(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen);
/** SDK注册函数，用于注册IIC接受与发送函数 **/
int32_t WitI2cFuncRegister(WitI2cWrite write_func, WitI2cRead read_func);

/* can function */
/** 函数指针：can协议发送函数的抽象 **/
typedef void (*CanWrite)(uint8_t ucStdId, uint8_t *p_ucData, uint32_t uiLen);
/** SDK注册函数：用于注册can协议写函数 **/
int32_t WitCanWriteRegister(CanWrite write_func);

/* Delayms function */

/** 函数指针：毫秒级延时 **/
typedef void (*DelaymsCb)(uint16_t ucMs);

/** SDK注册函数，注册毫秒级延时函数 **/
int32_t WitDelayMsRegister(DelaymsCb delayms_func);

/** can协议读取函数 **/
void WitCanDataIn(uint8_t ucData[8], uint8_t ucLen);


/** 回调函数：寄存器值更新时调用 */
typedef void (*RegUpdateCb)(uint32_t uiReg, uint32_t uiRegNum);

/** SDK注册函数，注册寄存器更新回调函数 */
int32_t WitRegisterCallBack(RegUpdateCb update_func);

/** 写寄存器函数 */
int32_t WitWriteReg(uint32_t uiReg, uint16_t usData);

/** 读寄存器函数 */
int32_t WitReadReg(uint32_t uiReg, uint32_t uiReadNum);

/** 初始化函数：定义协议类型与设备通讯地址（默认为0x50） */
int32_t WitInit(uint32_t uiProtocol, uint8_t ucAddr);

/** 卸载函数 */
void WitDeInit(void);


/**
  ******************************************************************************
  * @file    wit_c_sdk.h
  * @author  Wit
  * @version V1.0
  * @date    05-May-2022
  * @brief   This file provides all Configure sensor function.
  ******************************************************************************
  * @attention
  *
  *        http://wit-motion.cn/
  *
  ******************************************************************************
  */

/** 加速度校准开始 **/
int32_t WitStartAccCali(void);
/** 加速度校准结束 **/
int32_t WitStopAccCali(void);

/** 磁场校准开始 **/
int32_t WitStartMagCali(void);
/** 磁场校准结束 **/
int32_t WitStopMagCali(void);

/** 设置串口协议波特率 **/
int32_t WitSetUartBaud(int32_t uiBaudIndex);
/** 设置带宽 **/
int32_t WitSetBandwidth(int32_t uiBaudWidth);
/** 设置回传率 **/
int32_t WitSetOutputRate(int32_t uiRate);
/** 设置输出内容 **/
int32_t WitSetContent(int32_t uiRsw);
/** 设置CAN协议波特率 **/
int32_t WitSetCanBaud(int32_t uiBaudIndex);

/** z角度校准开始 **/
int32_t WitStartANGLEZCali(void);
/** z角度校准结束 **/
int32_t WitStopANGLEZCali(void);

/** 角度零点校准开始（校准零漂？） **/
int32_t WitStartREFANGLECali(void);
/** 角度零点校准结束（校准零漂？） **/
int32_t WitStopREFANGLECali(void);

/** 6轴算法校准开始 **/
int32_t WitStartALGRITHM6Cali(void);
/** 6轴算法校准结束 **/
int32_t WitStopALGRITHM6Cali(void);

/** 零漂校准开始？ **/
int32_t WitStartRKMODECali(void);
/** 零漂校准结束？ **/
int32_t WitStopRKMODECali(void);

/** Yaw角校准开始 **/
int32_t WitStartIYAWCali(void);

/** 检验数据有效性（有效范围） **/
char CheckRange(short sTemp, short sMin, short sMax);

/** 全局变量，用于存储所有读取到的寄存器的值 **/
extern int16_t sReg[REGSIZE];

#ifdef __cplusplus
}
#endif

#endif /* __WIT_C_SDK_H */
