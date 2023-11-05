#ifndef __IIC_H
#define __IIC_H

#include "stm32f1xx_hal.h"

	   		   

/***********************************************
 * iic移植调整 https://www.shuzhiduo.com/A/KE5QAqXP5L/
 *********************************************/
#define MPU_IIC_GPIO   GPIOB 		
#define MPU_IIC_SCL    GPIO_PIN_6 		
#define MPU_IIC_SDA    GPIO_PIN_7 		
#define MPU_IIC_DELAY  2



 /***********************************************
 * io口封装
 *********************************************/
#define I2C_SDA_UP        HAL_GPIO_WritePin(MPU_IIC_GPIO,MPU_IIC_SDA,GPIO_PIN_SET);                //SDA高电平
#define I2C_SDA_LOW        HAL_GPIO_WritePin(MPU_IIC_GPIO,MPU_IIC_SDA,GPIO_PIN_RESET);            //SDA低电平
#define I2C_SCL_UP        HAL_GPIO_WritePin(MPU_IIC_GPIO,MPU_IIC_SCL,GPIO_PIN_SET);          //SCL高电平
#define I2C_SCL_LOW        HAL_GPIO_WritePin(MPU_IIC_GPIO,MPU_IIC_SCL,GPIO_PIN_RESET);          //SCL低电平
#define I2C_SDA_READ       HAL_GPIO_ReadPin(MPU_IIC_GPIO, MPU_IIC_SDA);

//IIC所有操作函数
void MPU_IIC_Delay(void);				//MPU IIC延时函数
void delay_us(uint32_t us);
/***********************************************
 * 初始化全部上拉
 *********************************************/
void SDA_OUT(void);              
void SDA_IN(void);                
void SCL_OUT(void);                
void SCL_IN(void);                

void MPU_IIC_Start(void);				//发送IIC开始信号
void MPU_IIC_Stop(void);	  			//发送IIC停止信号
void MPU_IIC_Send_Byte(uint8_t txd);			//IIC发送一个字节
uint8_t MPU_IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
uint8_t MPU_IIC_Wait_Ack(void); 				//IIC等待ACK信号
void MPU_IIC_Ack(void);					//IIC发送ACK信号
void MPU_IIC_NAck(void);				//IIC不发送ACK信号
#endif

