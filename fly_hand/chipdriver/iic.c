#include "iic.h"
 
  //MPU IIC ��ʱ����
void MPU_IIC_Delay(void)
{
	delay_us(MPU_IIC_DELAY);
}

void SDA_OUT(void){
	GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = MPU_IIC_SDA ;                     
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MPU_IIC_GPIO, &GPIO_InitStruct);
}
void SDA_IN(void){
	GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = MPU_IIC_SDA ;                     
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MPU_IIC_GPIO, &GPIO_InitStruct);
}
void SCL_OUT(void){
	GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = MPU_IIC_SCL;                     
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MPU_IIC_GPIO, &GPIO_InitStruct);
}
void SCL_IN(void){
	GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = MPU_IIC_SCL;                     
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MPU_IIC_GPIO, &GPIO_InitStruct);
}

//����IIC��ʼ�ź�
void MPU_IIC_Start(void)
{	
	  SDA_OUT();
  	SCL_OUT(); // todo һ��û�������
    HAL_GPIO_WritePin(MPU_IIC_GPIO,MPU_IIC_SDA,GPIO_PIN_SET); 
    HAL_GPIO_WritePin(MPU_IIC_GPIO,MPU_IIC_SCL,GPIO_PIN_SET);
    MPU_IIC_Delay();
    I2C_SDA_LOW;        
    MPU_IIC_Delay();
    I2C_SCL_LOW;      
	  MPU_IIC_Delay();
}	  
//����IICֹͣ�ź�
void MPU_IIC_Stop(void)
{
	SDA_OUT();//sda�����
	SCL_OUT(); // todo һ��û�������
	I2C_SCL_LOW;
	I2C_SDA_LOW;
 	MPU_IIC_Delay();
	I2C_SCL_UP;
	I2C_SDA_UP;
	MPU_IIC_Delay();							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
uint8_t MPU_IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����  
	I2C_SDA_UP;
	MPU_IIC_Delay();	   
	I2C_SCL_UP;
	MPU_IIC_Delay();	 
	GPIO_PinState pinStatus = I2C_SDA_READ;
	while(pinStatus == GPIO_PIN_SET)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			MPU_IIC_Stop();
			return 1;
		}
	}
	I2C_SCL_LOW;//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
void MPU_IIC_Ack(void)
{
	I2C_SCL_LOW;
	SDA_OUT();
	I2C_SDA_LOW;
	MPU_IIC_Delay();
	I2C_SCL_UP;
	MPU_IIC_Delay();
	I2C_SCL_LOW;
}
//������ACKӦ��		    
void MPU_IIC_NAck(void)
{
	I2C_SCL_LOW;
	SDA_OUT();
	I2C_SDA_UP;
	MPU_IIC_Delay();
	I2C_SCL_UP;
	MPU_IIC_Delay();
	I2C_SCL_LOW;
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void MPU_IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
		SDA_OUT(); 	    
    I2C_SCL_LOW;
    for(t=0;t<8;t++)
    {              
				if((txd&0x80)>>7){
					I2C_SDA_UP;
				}else{
					I2C_SDA_LOW;
				}
				txd<<=1; 	  
				I2C_SCL_UP;
				MPU_IIC_Delay(); 
				I2C_SCL_LOW;	
				MPU_IIC_Delay();
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 MPU_IIC_Read_Byte(unsigned char ack)
{
		unsigned char i,receive=0;
		SDA_IN();//SDA����Ϊ����
		for(i=0;i<8;i++ ){
				I2C_SCL_LOW; 
				MPU_IIC_Delay();
				I2C_SCL_UP;
				receive<<=1;
				GPIO_PinState pinStatus = I2C_SDA_READ;
				if(pinStatus){
					  receive++;   
				}
				MPU_IIC_Delay(); 
		}					 
		if (!ack)
				MPU_IIC_NAck();//����nACK
		else
				MPU_IIC_Ack(); //����ACK   
		return receive;
}

