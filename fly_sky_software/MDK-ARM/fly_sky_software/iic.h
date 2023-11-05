#ifndef __IIC_H
#define __IIC_H

#include "stm32f1xx_hal.h"

	   		   

/***********************************************
 * iic��ֲ���� https://www.shuzhiduo.com/A/KE5QAqXP5L/
 *********************************************/
#define MPU_IIC_GPIO   GPIOB 		
#define MPU_IIC_SCL    GPIO_PIN_6 		
#define MPU_IIC_SDA    GPIO_PIN_7 		
#define MPU_IIC_DELAY  2



 /***********************************************
 * io�ڷ�װ
 *********************************************/
#define I2C_SDA_UP        HAL_GPIO_WritePin(MPU_IIC_GPIO,MPU_IIC_SDA,GPIO_PIN_SET);                //SDA�ߵ�ƽ
#define I2C_SDA_LOW        HAL_GPIO_WritePin(MPU_IIC_GPIO,MPU_IIC_SDA,GPIO_PIN_RESET);            //SDA�͵�ƽ
#define I2C_SCL_UP        HAL_GPIO_WritePin(MPU_IIC_GPIO,MPU_IIC_SCL,GPIO_PIN_SET);          //SCL�ߵ�ƽ
#define I2C_SCL_LOW        HAL_GPIO_WritePin(MPU_IIC_GPIO,MPU_IIC_SCL,GPIO_PIN_RESET);          //SCL�͵�ƽ
#define I2C_SDA_READ       HAL_GPIO_ReadPin(MPU_IIC_GPIO, MPU_IIC_SDA);

//IIC���в�������
void MPU_IIC_Delay(void);				//MPU IIC��ʱ����
void delay_us(uint32_t us);
/***********************************************
 * ��ʼ��ȫ������
 *********************************************/
void SDA_OUT(void);              
void SDA_IN(void);                
void SCL_OUT(void);                
void SCL_IN(void);                

void MPU_IIC_Start(void);				//����IIC��ʼ�ź�
void MPU_IIC_Stop(void);	  			//����IICֹͣ�ź�
void MPU_IIC_Send_Byte(uint8_t txd);			//IIC����һ���ֽ�
uint8_t MPU_IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
uint8_t MPU_IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void MPU_IIC_Ack(void);					//IIC����ACK�ź�
void MPU_IIC_NAck(void);				//IIC������ACK�ź�
#endif

