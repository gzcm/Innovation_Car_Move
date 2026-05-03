#include "wit_c_sdk.h"

/** 串口发送函数 **/
static SerialWrite p_WitSerialWriteFunc = NULL;
/** IIC发送函数 **/
static WitI2cWrite p_WitI2cWriteFunc = NULL;
/** IIC读取函数 **/
static WitI2cRead p_WitI2cReadFunc = NULL;
/** can发送函数 **/
static CanWrite p_WitCanWriteFunc = NULL;
/** 寄存器更新回调函数（存疑） **/
static RegUpdateCb p_WitRegUpdateCbFunc = NULL;
/** 毫秒级延时函数 **/
static DelaymsCb p_WitDelaymsFunc = NULL;

/** IIC/Can协议使用时的设备地址 **/
static uint8_t s_ucAddr = 0xff;
/** 数据缓存区 **/
static uint8_t s_ucWitDataBuff[WIT_DATA_BUFF_SIZE];
/** 读取数据时的地址偏移量		协议指示		读取到的寄存器的索引 **/
static uint32_t s_uiWitDataCnt = 0, s_uiProtoclo = 0, s_uiReadRegIndex = 0;
int16_t sReg[REGSIZE];

/** modbus 协议的写指令 **/
#define FuncW 0x06
/** modbus 协议的读指令 **/
#define FuncR 0x03

/** 查表法生成CRC校验（高位） **/
static const uint8_t __auchCRCHi[256] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40
};

/** 查表法生成CRC校验（低位位） **/
static const uint8_t __auchCRCLo[256] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
    0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
    0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
    0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
    0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
    0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
    0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
    0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
    0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
    0x40
};


/**
 * Modbus CRC16 校验函数
 * @param puchMsg
 * @param usDataLen
 * @return 16位校验码
 */
static uint16_t __CRC16(uint8_t *puchMsg, uint16_t usDataLen)
{
    uint8_t uchCRCHi = 0xFF;
    uint8_t uchCRCLo = 0xFF;
    uint8_t uIndex;
    int i = 0;
    uchCRCHi = 0xFF;
    uchCRCLo = 0xFF;
    for (; i<usDataLen; i++)
    {
        uIndex = uchCRCHi ^ puchMsg[i];
        uchCRCHi = uchCRCLo ^ __auchCRCHi[uIndex];
        uchCRCLo = __auchCRCLo[uIndex] ;
    }
    return (uint16_t)(((uint16_t)uchCRCHi << 8) | (uint16_t)uchCRCLo) ;
}

/**
 * 累加值校验码
 * @param data
 * @param len
 * @return 8位校验码
 */
static uint8_t __CaliSum(uint8_t *data, uint32_t len)
{
    uint32_t i;
    uint8_t ucCheck = 0;
    for(i=0; i<len; i++) ucCheck += *(data + i);
    return ucCheck;
}

/**
 * 串口写函数注册函数
 * @param Write_func 串口发送函数指针
 ** @return 程序状态（无效值/无错误）
 */
int32_t WitSerialWriteRegister(SerialWrite Write_func)
{
    if(!Write_func)return WIT_HAL_INVAL;
    p_WitSerialWriteFunc = Write_func;
    return WIT_HAL_OK;
}

/**
 * 数据处理函数（下方注释有误？）
 * @param ucIndex 传入读协议的第二位，表示数据类型
 * @param p_data  由于接受数据的数组的指针（建议长度为4)
 * @param uiLen	指定接受数据的个数
 */
static void CopeWitData(uint8_t ucIndex, uint16_t *p_data, uint32_t uiLen)
{
	//定义对应的寄存器地址与对应的数据长度，例如角度输出协议WITANGLE的地址为0x3d,共三个角度可查；接着为版本号，一个字节；
    uint32_t uiReg1 = 0, uiReg2 = 0, uiReg1Len = 0, uiReg2Len = 0;
    uint16_t *p_usReg1Val = p_data;
    uint16_t *p_usReg2Val = p_data+3;

	//一般来说仅需要解码一个寄存器的数据即可，即为4个字节
    uiReg1Len = 4;
    switch(ucIndex)
    {
        case WIT_ACC:   uiReg1 = AX;    uiReg1Len = 3;  uiReg2 = TEMP;  uiReg2Len = 1;  break;
        case WIT_ANGLE: uiReg1 = Roll;  uiReg1Len = 3;  uiReg2 = VERSION;  uiReg2Len = 1;  break;
        case WIT_TIME:  uiReg1 = YYMM;	break;
        case WIT_GYRO:  uiReg1 = GX;  uiLen = 3;break;
        case WIT_MAGNETIC: uiReg1 = HX;  uiLen = 3;break;
        case WIT_DPORT: uiReg1 = D0Status;  break;
        case WIT_PRESS: uiReg1 = PressureL;  break;
        case WIT_GPS:   uiReg1 = LonL;  break;
        case WIT_VELOCITY: uiReg1 = GPSHeight;  break;
        case WIT_QUATER:    uiReg1 = q0;  break;
        case WIT_GSA:   uiReg1 = SVNUM;  break;
        case WIT_REGVALUE:  uiReg1 = s_uiReadRegIndex;  break;
		default:
			return ;

    }

    if(uiLen == 3)
    {
    	//如果需要的数据仅为3个则只需要读取头三个字节一般为三个角度
        uiReg1Len = 3;
        uiReg2Len = 0;
    }
	//读取寄存器1返回的数据
    if(uiReg1Len)
	{
		memcpy(&sReg[uiReg1], p_usReg1Val, uiReg1Len<<1);
		p_WitRegUpdateCbFunc(uiReg1, uiReg1Len);
	}
	//读取寄存器2返回的数据
    if(uiReg2Len)
	{
		memcpy(&sReg[uiReg2], p_usReg2Val, uiReg2Len<<1);
		p_WitRegUpdateCbFunc(uiReg2, uiReg2Len);
	}
}

/**
 * 处理接收到的单字节数据，将数据拼接成完整帧并解析
 * @param ucData 串口接收到的单字节数据
 *
 * 执行逻辑：接受ucdata，存储到s_ucWitDataBuff,退出；直到满11个字节对比CRC，
 * 如果不符合，则丢掉帧头，触发第一个字节不是0x55的条件，一直丢弃帧头，直到下一个数据帧传入。
 * 如果符合，表示接受无误，将数据传给CopeWitData函数进行处理，接着清空缓存区准备下一次数据接受
 *
 * 使用要点：传入串口接受到的一字节数据即可
*/
void WitSerialDataIn(uint8_t ucData)
{
    uint16_t usCRC16, usTemp, i, usData[4];
    uint8_t ucSum;

    if(p_WitRegUpdateCbFunc == NULL)return ;
    s_ucWitDataBuff[s_uiWitDataCnt++] = ucData;
    switch(s_uiProtoclo)
    {
    case WIT_PROTOCOL_NORMAL:// 普通协议数据处理
    		// 数据帧第一个字节必须是 0x55，否者缓冲区向前移一位（丢弃第一个）
            if(s_ucWitDataBuff[0] != 0x55)
            {
                s_uiWitDataCnt--;
                memcpy(s_ucWitDataBuff, &s_ucWitDataBuff[1], s_uiWitDataCnt);
                return ;
            }
    		// 收满11字节数据才处理
            if(s_uiWitDataCnt >= 11)
            {
            	// 校验和 = 前10字节相加低8位
                ucSum = __CaliSum(s_ucWitDataBuff, 10);
                if(ucSum != s_ucWitDataBuff[10])
                {
                    s_uiWitDataCnt--;
                    memcpy(s_ucWitDataBuff, &s_ucWitDataBuff[1], s_uiWitDataCnt);
                    return ;
                }
            	// 将数据低高字节组合成16位数据
                usData[0] = ((uint16_t)s_ucWitDataBuff[3] << 8) | (uint16_t)s_ucWitDataBuff[2];
                usData[1] = ((uint16_t)s_ucWitDataBuff[5] << 8) | (uint16_t)s_ucWitDataBuff[4];
                usData[2] = ((uint16_t)s_ucWitDataBuff[7] << 8) | (uint16_t)s_ucWitDataBuff[6];
                usData[3] = ((uint16_t)s_ucWitDataBuff[9] << 8) | (uint16_t)s_ucWitDataBuff[8];

            	// 解析数据（通知回调），传入数据类型，存放位置与数据个数
                CopeWitData(s_ucWitDataBuff[1], usData, 4);

            	// 清空缓冲区计数，准备接收下一帧数据
                s_uiWitDataCnt = 0;
            }
        break;
        case WIT_PROTOCOL_MODBUS:	// Modbus协议处理
            if(s_uiWitDataCnt > 2)
            {
                if(s_ucWitDataBuff[1] != FuncR)
                {
                    s_uiWitDataCnt--;
                    memcpy(s_ucWitDataBuff, &s_ucWitDataBuff[1], s_uiWitDataCnt);
                    return ;
                }
                if(s_uiWitDataCnt < (s_ucWitDataBuff[2] + 5))return ;
                usTemp = ((uint16_t)s_ucWitDataBuff[s_uiWitDataCnt-2] << 8) | s_ucWitDataBuff[s_uiWitDataCnt-1];
                usCRC16 = __CRC16(s_ucWitDataBuff, s_uiWitDataCnt-2);
                if(usTemp != usCRC16)
                {
                    s_uiWitDataCnt--;
                    memcpy(s_ucWitDataBuff, &s_ucWitDataBuff[1], s_uiWitDataCnt);
                    return ;
                }
                usTemp = s_ucWitDataBuff[2] >> 1;
                for(i = 0; i < usTemp; i++)
                {
                    sReg[i+s_uiReadRegIndex] = ((uint16_t)s_ucWitDataBuff[(i<<1)+3] << 8) | s_ucWitDataBuff[(i<<1)+4];
                }
                p_WitRegUpdateCbFunc(s_uiReadRegIndex, usTemp);
                s_uiWitDataCnt = 0;
            }
        break;
        case WIT_PROTOCOL_CAN:
        case WIT_PROTOCOL_I2C:
        s_uiWitDataCnt = 0;
        break;
    }
	// 防止缓冲区溢出，超过最大长度时清零
    if(s_uiWitDataCnt == WIT_DATA_BUFF_SIZE)s_uiWitDataCnt = 0;
}

/**
 * @brief 注册I2C读写函数指针给SDK
 *
 * 通过此函数，将用户实现的I2C写入和读取函数注册到SDK中，
 * 使SDK内部能够通过这些函数进行底层I2C通讯操作。
 *
 * @param write_func 用户实现的I2C写函数指针，类型为WitI2cWrite
 * @param read_func 用户实现的I2C读函数指针，类型为WitI2cRead
 *
 * @return int32_t 返回状态码
 *         - WIT_HAL_OK (0) 注册成功
 *         - WIT_HAL_INVAL (-1) 传入的函数指针无效（NULL）
 */
int32_t WitI2cFuncRegister(WitI2cWrite write_func, WitI2cRead read_func)
{
    if(!write_func)return WIT_HAL_INVAL;
    if(!read_func)return WIT_HAL_INVAL;
    p_WitI2cWriteFunc = write_func;
    p_WitI2cReadFunc = read_func;
    return WIT_HAL_OK;
}

/**
 * @brief 注册CAN写函数指针给SDK
 *
 * 通过此函数，将用户实现的CAN写入函数注册到SDK中，
 * 使SDK能够通过该函数实现底层CAN通信写操作。
 *
 * @param Write_func 用户实现的CAN写函数指针，类型为CanWrite
 *
 * @return int32_t 返回状态码
 *         - WIT_HAL_OK (0) 注册成功
 *         - WIT_HAL_INVAL (-1) 传入的函数指针无效（NULL）
 */
int32_t WitCanWriteRegister(CanWrite Write_func)
{
    if(!Write_func)return WIT_HAL_INVAL;
    p_WitCanWriteFunc = Write_func;
    return WIT_HAL_OK;
}


void WitCanDataIn(uint8_t ucData[8], uint8_t ucLen)
{
	uint16_t usData[3];
    if(p_WitRegUpdateCbFunc == NULL)return ;
    if(ucLen < 8)return ;
    switch(s_uiProtoclo)
    {
        case WIT_PROTOCOL_CAN:
            if(ucData[0] != 0x55)return ;
            usData[0] = ((uint16_t)ucData[3] << 8) | ucData[2];
            usData[1] = ((uint16_t)ucData[5] << 8) | ucData[4];
            usData[2] = ((uint16_t)ucData[7] << 8) | ucData[6];
            CopeWitData(ucData[1], usData, 3);
            break;
        case WIT_PROTOCOL_NORMAL:
        case WIT_PROTOCOL_MODBUS:
        case WIT_PROTOCOL_I2C:
            break;
    }
}

/**
 * @brief 注册寄存器更新回调函数
 *
 * 通过此函数，将用户实现的寄存器值更新回调函数注册到SDK，
 * 当寄存器数据发生变化时，SDK会调用该回调函数通知用户。
 *
 * @param update_func 用户实现的回调函数指针，类型为RegUpdateCb
 *
 * @return int32_t 返回状态码
 *         - WIT_HAL_OK (0) 注册成功
 *         - WIT_HAL_INVAL (-1) 传入的函数指针无效（NULL）
 */
int32_t WitRegisterCallBack(RegUpdateCb update_func)
{
    if(!update_func)return WIT_HAL_INVAL;
    p_WitRegUpdateCbFunc = update_func;
    return WIT_HAL_OK;
}

/**
 * @brief 向指定寄存器写入数据
 *
 * 根据当前通信协议，通过对应接口向指定寄存器写入16位数据。
 *
 * @param uiReg 寄存器地址索引，范围应小于REGSIZE
 * @param usData 待写入的数据（16位）
 *
 * @return int32_t 返回状态码
 *         - WIT_HAL_OK (0) 写入成功
 *         - WIT_HAL_INVAL (-1) 寄存器地址超出范围或协议无效
 *         - WIT_HAL_EMPTY (-2) 串口、CAN或I2C写函数未注册
 */
int32_t WitWriteReg(uint32_t uiReg, uint16_t usData)
{
    uint16_t usCRC;
    uint8_t ucBuff[8];

	// 1. 先检查寄存器地址是否越界
    if(uiReg >= REGSIZE)return WIT_HAL_INVAL;

	// 2. 根据当前协议类型，采用不同的写数据格式
    switch(s_uiProtoclo)
    {
        case WIT_PROTOCOL_NORMAL:
    		// 普通串口协议
    	if(p_WitSerialWriteFunc == NULL)return WIT_HAL_EMPTY; // 确保写函数已绑定

    		// 构造写数据包：0xFF, 0xAA, 寄存器地址, 数据低字节, 数据高字节
            ucBuff[0] = 0xFF;
            ucBuff[1] = 0xAA;
            ucBuff[2] = uiReg & 0xFF;
            ucBuff[3] = usData & 0xff;
            ucBuff[4] = usData >> 8;

    		// 调用写函数发送数据
            p_WitSerialWriteFunc(ucBuff, 5);
            break;
        case WIT_PROTOCOL_MODBUS:
            if(p_WitSerialWriteFunc == NULL)return WIT_HAL_EMPTY;
            ucBuff[0] = s_ucAddr;
            ucBuff[1] = FuncW;
            ucBuff[2] = uiReg >> 8;
            ucBuff[3] = uiReg & 0xFF;
            ucBuff[4] = usData >> 8;
            ucBuff[5] = usData & 0xff;
            usCRC = __CRC16(ucBuff, 6);
            ucBuff[6] = usCRC >> 8;
            ucBuff[7] = usCRC & 0xff;
            p_WitSerialWriteFunc(ucBuff, 8);
            break;
        case WIT_PROTOCOL_CAN:
            if(p_WitCanWriteFunc == NULL)return WIT_HAL_EMPTY;
            ucBuff[0] = 0xFF;
            ucBuff[1] = 0xAA;
            ucBuff[2] = uiReg & 0xFF;
            ucBuff[3] = usData & 0xff;
            ucBuff[4] = usData >> 8;
            p_WitCanWriteFunc(s_ucAddr, ucBuff, 5);
            break;
        case WIT_PROTOCOL_I2C:
            if(p_WitI2cWriteFunc == NULL)return WIT_HAL_EMPTY;
            ucBuff[0] = usData & 0xff;
            ucBuff[1] = usData >> 8;
			if(p_WitI2cWriteFunc(s_ucAddr << 1, uiReg, ucBuff, 2) != 1)
			{
				//printf("i2c write fail\r\n");
			}
        break;
	default:
    		// 不支持的协议
            return WIT_HAL_INVAL;
    }
	// 正常完成返回成功
    return WIT_HAL_OK;
}

/**
 * @brief 读取指定寄存器的数据
 *
 * 根据当前通信协议（普通协议、Modbus、CAN或I2C），
 * 构造对应的读取命令并发送，通过回调函数接收数据或处理读写结果。
 *
 * @param uiReg       起始寄存器地址（索引）
 * @param uiReadNum   需要读取的寄存器数量
 *
 * @return WIT_HAL_OK       读取命令发送成功
 * @return WIT_HAL_INVAL    参数无效（寄存器越界或读取数量超限）
 * @return WIT_HAL_EMPTY    发送函数未注册或不可用
 * @return WIT_HAL_NOMEM    缓冲区内存不足
 *
 * @note 读取到的数据通过注册的回调函数通知上层应用（仅I2C协议主动回调）
 */
int32_t WitReadReg(uint32_t uiReg, uint32_t uiReadNum)
{
    uint16_t usTemp, i;
    uint8_t ucBuff[8];

	// 1. 检查寄存器范围是否合法（起始寄存器 + 读取数量不能超过寄存器最大数）
    if((uiReg + uiReadNum) >= REGSIZE)return WIT_HAL_INVAL;

	// 2. 根据当前协议类型，执行对应读取操作
    switch(s_uiProtoclo)
    {
        case WIT_PROTOCOL_NORMAL:
    		// 普通协议下，最多读4个寄存器
            if(uiReadNum > 4)return WIT_HAL_INVAL;
            if(p_WitSerialWriteFunc == NULL)return WIT_HAL_EMPTY;

    		// 构造普通协议读命令包
            ucBuff[0] = 0xFF;			//协议头
            ucBuff[1] = 0xAA;			//协议头
            ucBuff[2] = 0x27;			//读取指令
            ucBuff[3] = uiReg & 0xff;	// 寄存器低字节
            ucBuff[4] = uiReg >> 8;		// 寄存器高字节

    		// 发送读命令
            p_WitSerialWriteFunc(ucBuff, 5);
            break;
        case WIT_PROTOCOL_MODBUS:
            if(p_WitSerialWriteFunc == NULL)return WIT_HAL_EMPTY;
            usTemp = uiReadNum << 1;
            if((usTemp + 5) > WIT_DATA_BUFF_SIZE)return WIT_HAL_NOMEM;
            ucBuff[0] = s_ucAddr;
            ucBuff[1] = FuncR;
            ucBuff[2] = uiReg >> 8;
            ucBuff[3] = uiReg & 0xFF;
            ucBuff[4] = uiReadNum >> 8;
            ucBuff[5] = uiReadNum & 0xff;
            usTemp = __CRC16(ucBuff, 6);
            ucBuff[6] = usTemp >> 8;
            ucBuff[7] = usTemp & 0xff;
            p_WitSerialWriteFunc(ucBuff, 8);
            break;
        case WIT_PROTOCOL_CAN:
            if(uiReadNum > 3)return WIT_HAL_INVAL;
            if(p_WitCanWriteFunc == NULL)return WIT_HAL_EMPTY;
            ucBuff[0] = 0xFF;
            ucBuff[1] = 0xAA;
            ucBuff[2] = 0x27;
            ucBuff[3] = uiReg & 0xff;
            ucBuff[4] = uiReg >> 8;
            p_WitCanWriteFunc(s_ucAddr, ucBuff, 5);
            break;
        case WIT_PROTOCOL_I2C:
            if(p_WitI2cReadFunc == NULL)return WIT_HAL_EMPTY;
            usTemp = uiReadNum << 1;
            if(WIT_DATA_BUFF_SIZE < usTemp)return WIT_HAL_NOMEM;
            if(p_WitI2cReadFunc(s_ucAddr << 1, uiReg, s_ucWitDataBuff, usTemp) == 1)
            {
                if(p_WitRegUpdateCbFunc == NULL)return WIT_HAL_EMPTY;
                for(i = 0; i < uiReadNum; i++)
                {
                    sReg[i+uiReg] = ((uint16_t)s_ucWitDataBuff[(i<<1)+1] << 8) | s_ucWitDataBuff[i<<1];
                }
                p_WitRegUpdateCbFunc(uiReg, uiReadNum);
            }
			
            break;
		default: 
            return WIT_HAL_INVAL;
    }

	// 保存读取起始寄存器索引，可能用于后续操作或回调
    s_uiReadRegIndex = uiReg;

    return WIT_HAL_OK;
}

/**
 * @brief 初始化Wit模块驱动
 *
 * 设置通信协议类型和设备地址，并清空数据计数器。
 *
 * @param uiProtocol   通信协议类型（例如WIT_PROTOCOL_NORMAL、WIT_PROTOCOL_MODBUS、WIT_PROTOCOL_CAN、WIT_PROTOCOL_I2C）
 * @param ucAddr       设备地址，通常用于I2C或Modbus协议，默认一般为0x50
 *
 * @return WIT_HAL_OK      初始化成功
 * @return WIT_HAL_INVAL   协议参数无效，超过定义范围
 */
int32_t WitInit(uint32_t uiProtocol, uint8_t ucAddr)
{
	// 检查协议是否在支持范围内，如果超出，返回参数无效
	if(uiProtocol > WIT_PROTOCOL_I2C)return WIT_HAL_INVAL;

	// 记录当前使用的通信协议
    s_uiProtoclo = uiProtocol;

	// 保存设备地址（通常用于I2C、Modbus或CAN通信中的设备标识）
    s_ucAddr = ucAddr;

	// 重置数据计数器，准备接收新数据
    s_uiWitDataCnt = 0;

	// 返回成功状态
    return WIT_HAL_OK;
}

/**
 * @brief 反初始化Wit模块驱动
 *
 * 清除所有通信接口函数指针和回调函数指针，
 * 重置设备地址、数据计数器及协议类型。
 *
 * 作用是释放资源、断开与外设的关联，恢复到初始状态。
 */
void WitDeInit(void)
{
	// 清除串口写函数指针，断开串口通信接口
	p_WitSerialWriteFunc = NULL;

	// 清除I2C写函数指针，断开I2C写通信接口
	p_WitI2cWriteFunc = NULL;

	// 清除I2C读函数指针，断开I2C读通信接口
	p_WitI2cReadFunc = NULL;

	// 清除CAN写函数指针，断开CAN通信接口
	p_WitCanWriteFunc = NULL;

	// 清除寄存器更新回调函数指针
	p_WitRegUpdateCbFunc = NULL;

	// 重置设备地址为无效值
	s_ucAddr = 0xff;

	// 重置数据计数器
	s_uiWitDataCnt = 0;

	// 重置协议类型
	s_uiProtoclo = 0;
}


/**
 * @brief 注册延时函数
 *
 * 该函数用于向SDK注册一个毫秒级延时函数指针，
 * 以便SDK内部在需要延时操作时调用用户实现的延时函数。
 *
 * @param delayms_func 用户提供的延时函数，函数原型为 void func(uint32_t ms)
 * @return int32_t 返回状态码
 *         - WIT_HAL_OK: 注册成功
 *         - WIT_HAL_INVAL: 传入的函数指针为空，注册失败
 */
int32_t WitDelayMsRegister(DelaymsCb delayms_func)
{
	if(!delayms_func) return WIT_HAL_INVAL;   // 如果传入的函数指针为空，返回错误码
	p_WitDelaymsFunc = delayms_func;          // 保存用户的延时函数指针
	return WIT_HAL_OK;                         // 注册成功返回OK
}

/**
 * @brief 检查给定数值是否在指定范围内
 *
 * 判断输入的有符号短整型数值 sTemp 是否位于闭区间 [sMin, sMax] 内。
 * 如果在范围内，返回 1（true），否则返回 0（false）。
 *
 * @param sTemp 待检测的数值
 * @param sMin  范围下限
 * @param sMax  范围上限
 * @return char 返回 1 表示在范围内，返回 0 表示超出范围
 */
char CheckRange(short sTemp, short sMin, short sMax)
{
	if ((sTemp >= sMin) && (sTemp <= sMax))
		return 1;
	else
		return 0;
}

/*Acceleration calibration demo*/
/**
 * @brief 开始加速度计校准
 *
 * 使用前，请确保设备水平放置，然后调用此函数启动加速度计校准流程。
 *
 * 具体流程：
 * 1. 向 KEY 寄存器写入解锁命令，允许写操作。
 * 2. 根据当前通信协议等待适当的延迟时间，确保写操作完成。
 * 3. 向 CALSW 寄存器写入启动加速度计校准的命令。
 *
 * @return int32_t 返回校准启动状态：
 *         - WIT_HAL_OK 表示启动成功
 *         - WIT_HAL_ERROR 表示启动失败（寄存器写入失败）
 */
int32_t WitStartAccCali(void)
{
	// 解锁寄存器，允许写操作
	if (WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)
		return WIT_HAL_ERROR;

	// 根据协议类型延迟不同时间，确保寄存器写入稳定
	if (s_uiProtoclo == WIT_PROTOCOL_MODBUS)
		p_WitDelaymsFunc(20);
	else if (s_uiProtoclo == WIT_PROTOCOL_NORMAL)
		p_WitDelaymsFunc(1);
	else
		;  // 其他协议暂时不处理延迟

	// 启动加速度计校准命令写入
	if (WitWriteReg(CALSW, CALGYROACC) != WIT_HAL_OK)
		return WIT_HAL_ERROR;

	return WIT_HAL_OK;
}

/**
 * @brief 停止加速度计校准并保存校准结果
 *
 * 该函数用于结束加速度计的校准过程，并将校准参数保存到设备中。
 *
 * 具体流程：
 * 1. 向 CALSW 寄存器写入 NORMAL，表示结束校准状态，回到正常工作模式。
 * 2. 根据通信协议等待适当的延迟时间，确保寄存器操作完成。
 * 3. 向 SAVE 寄存器写入保存参数命令，将校准数据写入设备存储。
 *
 * @return int32_t 返回停止校准状态：
 *         - WIT_HAL_OK 表示停止并保存成功
 *         - WIT_HAL_ERROR 表示操作失败（寄存器写入失败）
 */
int32_t WitStopAccCali(void)
{
	// 设置校准状态为正常，结束校准过程
	if (WitWriteReg(CALSW, NORMAL) != WIT_HAL_OK)
		return WIT_HAL_ERROR;

	// 根据协议延迟，确保操作生效
	if (s_uiProtoclo == WIT_PROTOCOL_MODBUS)
		p_WitDelaymsFunc(20);
	else if (s_uiProtoclo == WIT_PROTOCOL_NORMAL)
		p_WitDelaymsFunc(1);
	else
		; // 其他协议暂不处理

	// 保存校准参数
	if (WitWriteReg(SAVE, SAVE_PARAM) != WIT_HAL_OK)
		return WIT_HAL_ERROR;

	return WIT_HAL_OK;
}

/**
 * @brief 开始Z轴角度校准
 *
 * 本函数用于启动设备的Z轴角度校准流程。使用前请确保设备处于合适的校准状态。
 *
 * 具体步骤：
 * 1. 向 KEY 寄存器写入 KEY_UNLOCK，解锁相关寄存器以允许修改。
 * 2. 根据通信协议执行适当延迟，确保写操作生效。
 * 3. 向 CALSW 寄存器写入 CALANGLEZ，启动Z轴角度校准。
 *
 * @return int32_t
 *         - WIT_HAL_OK 表示成功启动校准
 *         - WIT_HAL_ERROR 表示写寄存器失败
 */
int32_t WitStartANGLEZCali(void)
{
	// 解锁寄存器，准备写入校准指令
	if (WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)
		return WIT_HAL_ERROR;

	// 根据协议延迟
	if (s_uiProtoclo == WIT_PROTOCOL_MODBUS)
		p_WitDelaymsFunc(20);
	else if (s_uiProtoclo == WIT_PROTOCOL_NORMAL)
		p_WitDelaymsFunc(1);
	else
		; // 其他协议暂不处理

	// 启动Z轴角度校准
	if (WitWriteReg(CALSW, CALANGLEZ) != WIT_HAL_OK)
		return WIT_HAL_ERROR;

	return WIT_HAL_OK;
}

/**
 * @brief 停止Z轴角度校准并保存参数
 *
 * 本函数用于停止设备的Z轴角度校准过程，并将校准结果保存到设备中。
 *
 * 具体步骤：
 * 1. 向 CALSW 寄存器写入 NORMAL，停止校准状态。
 * 2. 根据通信协议执行适当延迟，确保写操作生效。
 * 3. 向 SAVE 寄存器写入 SAVE_PARAM，保存当前参数到设备非易失存储。
 *
 * @return int32_t
 *         - WIT_HAL_OK 表示成功停止校准并保存参数
 *         - WIT_HAL_ERROR 表示写寄存器失败
 */
int32_t WitStopANGLEZCali(void)
{
	// 停止校准状态
	if(WitWriteReg(CALSW, NORMAL) != WIT_HAL_OK)
		return WIT_HAL_ERROR;

	// 根据协议延迟，确保设备处理完成
	if(s_uiProtoclo == WIT_PROTOCOL_MODBUS)
		p_WitDelaymsFunc(20);
	else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL)
		p_WitDelaymsFunc(1);
	else
		; // 其他协议暂不处理

	// 保存参数
	if(WitWriteReg(SAVE, SAVE_PARAM) != WIT_HAL_OK)
		return WIT_HAL_ERROR;

	return WIT_HAL_OK;
}

/**
 * @brief 启动角度零点校准（参考角度校准）
 *
 * 该函数用于启动设备的角度零点校准，通常用于校准设备的零漂，保证测量的角度基准准确。
 *
 * 具体流程：
 * 1. 向 KEY 寄存器写入解锁命令 KEY_UNLOCK，解锁寄存器以允许写操作。
 * 2. 根据当前使用的协议执行适当的延时，确保解锁命令生效。
 * 3. 向 CALSW 寄存器写入 CALREFANGLE，启动参考角度校准。
 *
 * @return int32_t
 *         - WIT_HAL_OK 表示成功启动校准
 *         - WIT_HAL_ERROR 表示写寄存器失败
 */
int32_t WitStartREFANGLECali(void)
{
	// 解锁寄存器
	if(WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)
		return WIT_HAL_ERROR;

	// 根据协议延时，确保解锁生效
	if(s_uiProtoclo == WIT_PROTOCOL_MODBUS)
		p_WitDelaymsFunc(20);
	else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL)
		p_WitDelaymsFunc(1);
	else
		; // 其他协议不处理

	// 启动参考角度校准
	if(WitWriteReg(CALSW, CALREFANGLE) != WIT_HAL_OK)
		return WIT_HAL_ERROR;

	return WIT_HAL_OK;
}

/**
 * @brief 停止角度零点校准（参考角度校准）并保存参数
 *
 * 该函数用于结束参考角度（零漂）校准过程，并将当前校准结果保存至设备内部存储。
 *
 * 具体流程：
 * 1. 向 CALSW 寄存器写入 NORMAL，停止当前的校准流程。
 * 2. 根据协议要求进行延时，保证写入操作的稳定性。
 * 3. 向 SAVE 寄存器写入 SAVE_PARAM，保存当前配置与校准数据。
 *
 * @return int32_t
 *         - WIT_HAL_OK 表示操作成功
 *         - WIT_HAL_ERROR 表示写寄存器失败
 */
int32_t WitStopREFANGLECali(void)
{
	// 1. 停止参考角度校准（将 CALSW 设置为 NORMAL 模式）
	if (WitWriteReg(CALSW, NORMAL) != WIT_HAL_OK)
		return WIT_HAL_ERROR;

	// 2. 根据通信协议延时（防止指令执行冲突）
	if (s_uiProtoclo == WIT_PROTOCOL_MODBUS)
		p_WitDelaymsFunc(20);
	else if (s_uiProtoclo == WIT_PROTOCOL_NORMAL)
		p_WitDelaymsFunc(1);
	else
		; // 其他协议无特殊处理

	// 3. 将校准参数保存到设备内部（非易失性存储）
	if (WitWriteReg(SAVE, SAVE_PARAM) != WIT_HAL_OK)
		return WIT_HAL_ERROR;

	return WIT_HAL_OK;
}

/**
 * @brief 启动 Z 轴（IYAW）角度归零校准
 *
 * 本函数用于触发陀螺仪进行 IYAW 校准操作，即将当前 Z 轴（偏航角）设为角度基准值 0°。
 *
 * 校准步骤如下：
 * 1. 向 KEY 寄存器写入 KEY_UNLOCK，以解锁写操作；
 * 2. 根据通信协议（MODBUS / NORMAL）插入必要延时，确保硬件响应；
 * 3. 向地址 0x76 寄存器写入 0x00，启动 IYAW 校准流程；
 *
 * 注意：校准过程中应确保传感器静止，以获得准确参考角度。
 *
 * @retval WIT_HAL_OK    校准命令已成功发出
 * @retval WIT_HAL_ERROR 解锁或写寄存器失败
 */
int32_t WitStartIYAWCali(void)
{
	// 解锁寄存器写权限
	if (WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)
		return WIT_HAL_ERROR;

	// 按照协议插入延时，确保写指令生效
	if (s_uiProtoclo == WIT_PROTOCOL_MODBUS)
		p_WitDelaymsFunc(20);
	else if (s_uiProtoclo == WIT_PROTOCOL_NORMAL)
		p_WitDelaymsFunc(1);

	// 写入寄存器 0x76，触发 IYAW 校准操作（Z 轴归零）
	if (WitWriteReg(0x76, 0x00) != WIT_HAL_OK)
		return WIT_HAL_ERROR;

	return WIT_HAL_OK;
}


/**
 * @brief 设置Z轴运行模式
 *
 * @param mode 模式值（0x00~0x03）
 *             - 0x00: 正常模式
 *             - 0x01: 求峰峰值
 *             - 0x02: 求零偏
 *             - 0x03: 求标度因素
 *
 * 示例：发送：FF AA  48 01 00（自动获取零偏）
 *
 * @return int32_t
 *         - WIT_HAL_OK 成功
 *         - WIT_HAL_ERROR 写寄存器失败
 */
int32_t WitStartRKMODECali(void)
{
	// 1. 解锁寄存器写操作权限
	if (WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)
		return WIT_HAL_ERROR;

	// 2. 根据协议进行延时，确保解锁生效
	if (s_uiProtoclo == WIT_PROTOCOL_MODBUS)
		p_WitDelaymsFunc(20);
	else if (s_uiProtoclo == WIT_PROTOCOL_NORMAL)
		p_WitDelaymsFunc(1);
	else
		; // 其他协议无需延时

	// 3. 向寄存器 0x48 写入 0x01，启动零偏校准（RK模式）
	if (WitWriteReg(0x48, 0x01) != WIT_HAL_OK)
		return WIT_HAL_ERROR;

	return WIT_HAL_OK;
}


/**
 * @brief 停止零漂（RK模式）校准
 *
 * 本函数用于结束当前正在进行的 RK 模式零漂校准过程。
 * 其原理是通过向控制寄存器写入值，关闭 Z 轴零偏校准功能。
 *
 * 操作步骤：
 * 1. 向 KEY 寄存器写入 KEY_UNLOCK，解锁写寄存器权限；
 * 2. 根据通信协议延时（MODBUS：20ms，NORMAL：1ms）；
 * 3. 向寄存器地址 0x48 写入 0x00，表示关闭 RK 模式；
 *
 * @retval WIT_HAL_OK    操作成功
 * @retval WIT_HAL_ERROR 写寄存器失败
 */
int32_t WitStopRKMODECali(void)
{
	// 解锁写寄存器权限
	if (WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)
		return WIT_HAL_ERROR;

	// 根据协议插入必要延时
	if (s_uiProtoclo == WIT_PROTOCOL_MODBUS)
		p_WitDelaymsFunc(20);
	else if (s_uiProtoclo == WIT_PROTOCOL_NORMAL)
		p_WitDelaymsFunc(1);

	// 写入寄存器 0x48，值为 0x00，停止零漂校准
	if (WitWriteReg(0x48, 0x00) != WIT_HAL_OK)
		return WIT_HAL_ERROR;

	return WIT_HAL_OK;
}

/**
 * @brief 启动 6 轴传感器算法校准
 *
 * 该函数用于启动六轴（加速度计 + 陀螺仪）组合算法的自动校准过程，
 * 用于提高传感器输出数据的精度。
 *
 * 操作流程：
 * 1. 向 KEY 寄存器写入 KEY_UNLOCK，解锁寄存器写入权限；
 * 2. 根据通信协议类型进行适当的延时（MODBUS 需更长延时）；
 * 3. 向 AXIS6 寄存器写入 ALGRITHM6 指令，开始算法级校准；
 *
 * @retval WIT_HAL_OK    启动成功
 * @retval WIT_HAL_ERROR 写寄存器失败或参数无效
 */
int32_t WitStartALGRITHM6Cali(void)
{
	// 解锁写寄存器权限
	if (WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)
		return WIT_HAL_ERROR;

	// 根据协议类型插入适当延时
	if (s_uiProtoclo == WIT_PROTOCOL_MODBUS)
		p_WitDelaymsFunc(20);
	else if (s_uiProtoclo == WIT_PROTOCOL_NORMAL)
		p_WitDelaymsFunc(1);

	// 向 AXIS6 寄存器写入启动指令，执行 6 轴算法校准
	if (WitWriteReg(AXIS6, ALGRITHM6) != WIT_HAL_OK)
		return WIT_HAL_ERROR;

	return WIT_HAL_OK;
}

/**
 * @brief 停止 6 轴算法校准并保存参数
 *
 * 该函数用于停止正在进行的六轴算法校准，并将校准结果保存至设备参数中。
 *
 * 操作流程：
 * 1. 向 CALSW 寄存器写入 NORMAL，停止校准模式；
 * 2. 根据通信协议类型进行适当延时，确保设备稳定响应；
 * 3. 向 SAVE 寄存器写入 SAVE_PARAM，保存当前参数；
 *
 * @retval WIT_HAL_OK    停止校准并保存参数成功
 * @retval WIT_HAL_ERROR 写寄存器失败或参数无效
 */
int32_t WitStopALGRITHM6Cali(void)
{
	// 停止校准，设置回正常模式
	if (WitWriteReg(CALSW, NORMAL) != WIT_HAL_OK)
		return WIT_HAL_ERROR;

	// 根据协议延时，确保操作生效
	if (s_uiProtoclo == WIT_PROTOCOL_MODBUS)
		p_WitDelaymsFunc(20);
	else if (s_uiProtoclo == WIT_PROTOCOL_NORMAL)
		p_WitDelaymsFunc(1);

	// 保存参数至设备
	if (WitWriteReg(SAVE, SAVE_PARAM) != WIT_HAL_OK)
		return WIT_HAL_ERROR;

	return WIT_HAL_OK;
}


/*Magnetic field calibration*/
/**
 * @brief 启动磁场校准（Magnetometer Calibration）
 *
 * 本函数用于开始设备的磁场校准过程，用于校正磁力计传感器。
 *
 * 执行流程：
 * 1. 解锁寄存器权限（写入 KEY_UNLOCK 到 KEY 寄存器）；
 * 2. 根据协议类型延时，确保解锁操作稳定；
 * 3. 向校准控制寄存器 CALSW 写入校准启动标志 CALMAGMM，开始磁场校准；
 *
 * @return int32_t
 *         - WIT_HAL_OK 表示成功启动磁场校准
 *         - WIT_HAL_ERROR 表示启动失败
 */
int32_t WitStartMagCali(void)
{
	// 解锁寄存器写权限
	if (WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)
		return WIT_HAL_ERROR;

	// 根据当前协议类型，延时等待寄存器生效
	if (s_uiProtoclo == WIT_PROTOCOL_MODBUS)
		p_WitDelaymsFunc(20);
	else if (s_uiProtoclo == WIT_PROTOCOL_NORMAL)
		p_WitDelaymsFunc(1);

	// 启动磁场校准
	if (WitWriteReg(CALSW, CALMAGMM) != WIT_HAL_OK)
		return WIT_HAL_ERROR;

	return WIT_HAL_OK;
}

/**
 * @brief 停止磁场校准（Magnetometer Calibration）
 *
 * 本函数用于结束设备的磁场校准过程，恢复到正常工作模式。
 *
 * 执行流程：
 * 1. 解锁寄存器权限（写入 KEY_UNLOCK 到 KEY 寄存器）；
 * 2. 根据协议类型延时，确保解锁操作稳定；
 * 3. 向校准控制寄存器 CALSW 写入正常模式标志 NORMAL，停止磁场校准；
 *
 * @return int32_t
 *         - WIT_HAL_OK 表示成功停止校准
 *         - WIT_HAL_ERROR 表示操作失败
 */
int32_t WitStopMagCali(void)
{
	// 解锁寄存器写权限
	if (WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)
		return WIT_HAL_ERROR;

	// 根据当前协议类型，延时等待寄存器生效
	if (s_uiProtoclo == WIT_PROTOCOL_MODBUS)
		p_WitDelaymsFunc(20);
	else if (s_uiProtoclo == WIT_PROTOCOL_NORMAL)
		p_WitDelaymsFunc(1);

	// 写入正常模式，停止磁场校准
	if (WitWriteReg(CALSW, NORMAL) != WIT_HAL_OK)
		return WIT_HAL_ERROR;

	return WIT_HAL_OK;
}

/*change Band*/
/**
 * @brief 设置串口波特率
 *
 * 该函数用于配置设备的串口通信波特率。
 *
 * @param[in] uiBaudIndex  波特率索引，必须在 WIT_BAUD_4800 到 WIT_BAUD_921600 范围内。
 *
 * @return int32_t
 *         - WIT_HAL_OK       设置成功
 *         - WIT_HAL_INVAL    参数非法，超出有效波特率范围
 *         - WIT_HAL_ERROR    寄存器写入失败
 */
int32_t WitSetUartBaud(int32_t uiBaudIndex)
{
	// 1. 检查波特率索引是否有效
	if(!CheckRange(uiBaudIndex, WIT_BAUD_4800, WIT_BAUD_921600))
	{
		return WIT_HAL_INVAL;
	}

	// 2. 写入解锁密钥，允许写寄存器
	if(WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)
		return WIT_HAL_ERROR;

	// 3. 根据协议延时，确保寄存器写入稳定
	if(s_uiProtoclo == WIT_PROTOCOL_MODBUS)
		p_WitDelaymsFunc(20);
	else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL)
		p_WitDelaymsFunc(1);
	else
		; // 其他协议不延时

	// 4. 写入波特率寄存器，完成波特率设置
	if(WitWriteReg(BAUD, uiBaudIndex) != WIT_HAL_OK)
		return WIT_HAL_ERROR;

	// 5. 返回成功
	return WIT_HAL_OK;
}

/*change Can Band*/
int32_t WitSetCanBaud(int32_t uiBaudIndex)
{
	if(!CheckRange(uiBaudIndex,CAN_BAUD_1000000,CAN_BAUD_3000))
	{
		return WIT_HAL_INVAL;
	}
	if(WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(s_uiProtoclo == WIT_PROTOCOL_MODBUS)	p_WitDelaymsFunc(20);
	else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(1);
	else ;
	if(WitWriteReg(BAUD, uiBaudIndex) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}
/*change Bandwidth*/
/**
 * @brief 设置传感器带宽
 *
 * 该函数用于配置设备的传感器带宽参数，
 * 以控制传感器数据的滤波频率，影响响应速度和噪声水平。
 *
 * @param[in] uiBaudWidth  带宽参数，必须在 BANDWIDTH_256HZ 到 BANDWIDTH_5HZ 范围内。
 *
 * @return int32_t
 *         - WIT_HAL_OK       设置成功
 *         - WIT_HAL_INVAL    参数非法，超出有效带宽范围
 *         - WIT_HAL_ERROR    寄存器写入失败
 */
int32_t WitSetBandwidth(int32_t uiBaudWidth)
{
	// 1. 检查带宽参数是否有效
	if(!CheckRange(uiBaudWidth, BANDWIDTH_256HZ, BANDWIDTH_5HZ))
	{
		return WIT_HAL_INVAL;
	}

	// 2. 解锁寄存器写权限
	if(WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)
		return WIT_HAL_ERROR;

	// 3. 根据协议类型延时，确保寄存器写入稳定
	if(s_uiProtoclo == WIT_PROTOCOL_MODBUS)
		p_WitDelaymsFunc(20);
	else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL)
		p_WitDelaymsFunc(1);
	else
		; // 其他协议不延时

	// 4. 写入带宽寄存器，完成带宽设置
	if(WitWriteReg(BANDWIDTH, uiBaudWidth) != WIT_HAL_OK)
		return WIT_HAL_ERROR;

	// 5. 返回成功
	return WIT_HAL_OK;
}


/*change output rate */
/**
 * @brief 设置传感器数据输出频率（回传率）
 *
 * 该函数用于配置传感器数据的输出速率，影响数据更新的频率。
 *
 * @param[in] uiRate  输出频率参数，必须在 RRATE_02HZ 到 RRATE_NONE 范围内。
 *
 * @return int32_t
 *         - WIT_HAL_OK       设置成功
 *         - WIT_HAL_INVAL    参数非法，超出有效输出率范围
 *         - WIT_HAL_ERROR    寄存器写入失败
 */
int32_t WitSetOutputRate(int32_t uiRate)
{
	// 1. 检查输出率参数是否合法
	if(!CheckRange(uiRate, RRATE_02HZ, RRATE_NONE))
	{
		return WIT_HAL_INVAL;
	}

	// 2. 解锁寄存器写权限
	if(WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)
		return WIT_HAL_ERROR;

	// 3. 根据协议类型延时，确保写入操作稳定
	if(s_uiProtoclo == WIT_PROTOCOL_MODBUS)
		p_WitDelaymsFunc(20);
	else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL)
		p_WitDelaymsFunc(1);
	else
		; // 其他协议不延时

	// 4. 写入输出率寄存器，设置数据输出频率
	if(WitWriteReg(RRATE, uiRate) != WIT_HAL_OK)
		return WIT_HAL_ERROR;

	// 5. 返回成功
	return WIT_HAL_OK;
}


/*change WitSetContent */
/**
 * @brief 设置传感器输出内容配置
 *
 * 该函数用于配置传感器输出的数据内容，即选择哪些数据项会被回传。
 *
 * @param[in] uiRsw  输出内容选择参数，必须在有效范围 RSW_TIME 到 RSW_MASK 之间。
 *
 * @return int32_t
 *         - WIT_HAL_OK       设置成功
 *         - WIT_HAL_INVAL    参数非法，超出有效范围
 *         - WIT_HAL_ERROR    寄存器写入失败
 */
int32_t WitSetContent(int32_t uiRsw)
{
	// 1. 检查参数范围是否合法
	if(!CheckRange(uiRsw, RSW_TIME, RSW_MASK))
	{
		return WIT_HAL_INVAL;
	}

	// 2. 解锁寄存器写权限
	if(WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)
		return WIT_HAL_ERROR;

	// 3. 根据协议延时，确保写入稳定
	if(s_uiProtoclo == WIT_PROTOCOL_MODBUS)
		p_WitDelaymsFunc(20);
	else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL)
		p_WitDelaymsFunc(1);
	else
		; // 其他协议无延时

	// 4. 写入寄存器，配置输出内容
	if(WitWriteReg(RSW, uiRsw) != WIT_HAL_OK)
		return WIT_HAL_ERROR;

	// 5. 返回成功状态
	return WIT_HAL_OK;
}



