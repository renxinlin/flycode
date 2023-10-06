#ifndef __COMMAND_H
#define __COMMAND_H
#include "command.h"
#include "nrf24l01.h"
#include "adc.h"



#define  LOW_SPEED_THRUST   (95.0)
#define  LOW_SPEED_PITCH    (10.0)
#define  LOW_SPEED_ROLL     (10.0)

#define  MID_SPEED_THRUST   (95.0)
#define  MID_SPEED_PITCH    (18.0)
#define  MID_SPEED_ROLL     (18.0)

#define  HIGH_SPEED_THRUST  (95.0)
#define  HIGH_SPEED_PITCH   (30.0)
#define  HIGH_SPEED_ROLL    (30.0)

#define  MIN_THRUST			(25.0)
#define  ALT_THRUST		    (50.0)
#define  MAX_YAW			(200.0)

static control_data flydata;


float limit(float value,float min, float max);
float limit(float value,float min, float max)
{
	if(value > max)
	{
		value = max;
	}
	else if(value < min)
	{
		value = min;
	}
	return value;
}

void commanderTask(void* param)
{
	float max_thrust = LOW_SPEED_THRUST;
	float max_pitch = LOW_SPEED_PITCH;
	float max_roll = LOW_SPEED_ROLL;
	control_data  percent;
	
	while(1)
	{
		HAL_Delay(10);
		max_thrust = LOW_SPEED_THRUST;
		max_pitch = LOW_SPEED_PITCH;
		max_roll = LOW_SPEED_ROLL;
		/*
		switch(configParam.flight.speed)
		{
			case LOW_SPEED:
				max_thrust = LOW_SPEED_THRUST;
				max_pitch = LOW_SPEED_PITCH;
				max_roll = LOW_SPEED_ROLL;
				break;
			case MID_SPEED:
				max_thrust = MID_SPEED_THRUST;
				max_pitch = MID_SPEED_PITCH;
				max_roll = MID_SPEED_ROLL;
				break;
			case HIGH_SPEED:
				max_thrust = HIGH_SPEED_THRUST;
				max_pitch = HIGH_SPEED_PITCH;
				max_roll = HIGH_SPEED_ROLL;
				break;
		}
		*/
		// ȥ�����м�������ͱ�Ե������
	//	ADCtoFlyDataPercent(&percent);
		
		//THRUST
		/*
		if(configParam.flight.ctrl == ALTHOLD_MODE || configParam.flight.ctrl == THREEHOLD_MODE)
		{
			flydata.thrust = percent.thrust * ALT_THRUST;
			flydata.thrust += ALT_THRUST;
			flydata.thrust = limit(flydata.thrust, 0, 100);
		}
		else
		{
			// �������ֶ�ģʽ
			flydata.thrust = percent.thrust * (max_thrust - MIN_THRUST);
			flydata.thrust += MIN_THRUST;
			flydata.thrust = limit(flydata.thrust, MIN_THRUST, max_thrust);
		}
		*/
		flydata.thrust = percent.thrust * ALT_THRUST;
		flydata.thrust += ALT_THRUST;
		flydata.thrust = limit(flydata.thrust, 0, 100);
		//ROLL
		flydata.roll = percent.roll * max_roll;
		flydata.roll = limit(flydata.roll, -max_roll, max_roll);
		//PITCH
		flydata.pitch = percent.pitch * max_pitch;
		flydata.pitch = limit(flydata.pitch, -max_pitch, max_pitch);
		//YAW
		flydata.yaw = percent.yaw * MAX_YAW;
		flydata.yaw = limit(flydata.yaw, -MAX_YAW, MAX_YAW);
		
		/*���ͷɿ�����*/
		if(RCLock==0)
		{	
			remoter_data send;
			send.flightMode = 0;
			/*
			switch(configParam.flight.mode)
			{
				case HEAD_LESS:
					send.flightMode = 1;
					break;
				case X_MODE:
					send.flightMode = 0;
					break;
			} */
			/*
			switch(configParam.flight.ctrl)
			{
				
				case ALTHOLD_MODE:
					send.ctrlMode = 1;
					break;
				case MANUAL_MODE:
					send.ctrlMode = 0;
					break;
				case THREEHOLD_MODE:
					send.ctrlMode = 3;
					break;
			}*/
			send.ctrlMode = 1;
			if(flydata.thrust<=MIN_THRUST && send.ctrlMode==0)
			{
				send.thrust = 0;
			}
			else
			{
				send.thrust = flydata.thrust;
			}
			
			if(TrimFlag == 1)
			{
				/*
				roll��������Ϊ0��
					����δ����У����Կ�����������־TrimFlagΪ�棨��΢��ģʽ��ʱ��pitch��roll��������Ϊ0������Ϊ������������������ȶ���ͣ״̬���Ա����΢����
					��΢��ģʽ�£�������Ҫ���������������΢С�ĵ����Ծ�������������̬�����pitch��roll����Ϊ0������������ܻ�����ƶ����Ӷ����µ���ʧ�顣����pitch��roll����Ϊ0������Ա�֤����������������ȶ���λ�ã��Ӷ������׽���΢����
					ֵ��ע����ǣ��ڷ�΢��ģʽ�£�TrimFlagΪ�٣���pitch��roll�Ǹ���ͨ����������ȡ�ķ����������ݽ������õģ��������ɽ���ƫ�ơ���ת�ȸ��ӵ��˶���
				*/
				send.pitch = 0;
				send.roll = 0;
			}
			else
			{
				send.pitch = flydata.pitch ;
				send.roll = flydata.roll;
			}
			send.yaw = flydata.yaw;
			send.trimPitch = configParam.trimP;
			send.trimRoll = configParam.trimR;
			
			/*���ͷɿ�����*/
			NRF24L01_TxPacket((u8*)&send);
		}
		
		/*����ң��������������λ��*/
		// todo ���Ƿ��͵�����
	}
}

#endif
