#ifndef FBM320_H
#define FBM320_H
#include "stm32f1xx.h"

//对 SENSER_OFFSET_FLAG 的位的操作
// SENSER_OFFSET_FLAG 八位标志位数组
#define SENSER_FLAG_SET(FLAG)   SENSER_OFFSET_FLAG|=FLAG                //标志位置1
#define SENSER_FLAG_RESET(FLAG) SENSER_OFFSET_FLAG&=~FLAG               //标志位值0  
#define GET_FLAG(FLAG)         (SENSER_OFFSET_FLAG&FLAG)==FLAG ? 1 : 0  //获取标志位状态

#define FBMAddr2 0x6C   //气压计从机IIC地址  110 1100 | 110 110?[?取决于sd0引脚,本硬件采用低电平]

/****************************************气压计寄存器地址***********************************************/
#define SPI_CTRL     0x00 //SPI通信时配置寄存器 (本工程使用IIC驱动 ，应设置为0x00)
#define FBM_ID       0x6B //FBM320 身份寄存器
#define FBM_VERSION   0xa5 //FBM320 版本不同的c0~12公式不同 参考github
#define FBM_COEFF1   0xAA //FBM320 校准寄存器
#define FBM_COEFF2   0xBB
#define FBM_COEFF3   0xD0
#define FBM_COEFF4   0xF1
#define FBM_RESET    0xE0 //FBM320 复位寄存器
#define FBM_CONFIG   0xF4 //FBM320 这配置寄存器 6:7 OSR（精度），0:5 101110:温度转化命令/110100:气压转换命令
#define DATA_MSB     0xF6 //FBM320 数据寄存器 16:23
#define DATA_CSB     0xF7 //FBM320 数据寄存器 8:15
#define DATA_LSB     0xF8 //FBM320 数据寄存器 0:7

#define OSR1024  0x00
#define OSR2048  0x40
#define OSR4096  0x80
#define OSR8192  0xC0
#define PRES_CONVERSION     0x34
#define TEMP_CONVERSION     0x2E
#define FBMRESET      0xB6
#define FBMID         0x42

extern float RPFilter;

/**
 * 复位fbm320并初始化fbm320的校准数据
 */
void FBM320_Init(void);

/**
 * 通过读取fbm320内部身份寄存器获取固定id确认硬件是否可用
 */
void FBM320_Check(void);

/*
 * 获取fbm320的校准数据
 *
 */
void FBM320_GetCoeff(void);

/*
* 根据压强和温度获取高度数据
 *
 */
void FBM320_Calculate(int32_t UP, int32_t UT);

/*
 * 计算海拔高度
 *
 */
int32_t Abs_Altitude(int32_t Press);

/*
* 获取初始所处位置高度作为相对高度0
 *
 */
uint8_t Init_Altitude(void);

/*
 * 获取相对高度=当前位置海拔高度-初始位置海拔高度
 *
 */
void FBM320_GetAltitude(void);

/*******************************************************
 *  iic通信
 *******************************************************
 */
uint8_t FBM_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf);
uint8_t FBM_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf);
uint8_t FBM320_ReadMultBytes(uint8_t reg,uint8_t len,uint8_t *buf);
uint8_t FBM320_WriteByte(uint8_t reg,uint8_t data);
uint8_t FBM320_ReadByte(uint8_t reg,uint8_t *buf);
uint8_t FBM320_WriteMultBytes(uint8_t reg,uint8_t len,uint8_t *buf);
uint8_t FBM320_getDeviceID(void);
uint8_t FBM320_testConnection(void);

#endif



  
