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
		// 去除了中间的死区和边缘的死区
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
			// 不考虑手动模式
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
		
		/*发送飞控数据*/
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
				roll都被设置为0？
					在这段代码中，可以看出当调整标志TrimFlag为真（即微调模式）时，pitch和roll都被设置为0。这是为了让四轴飞行器进入稳定悬停状态，以便进行微调。
					在微调模式下，往往需要对四轴飞行器进行微小的调整以纠正飞行器的姿态。如果pitch和roll都不为0，则飞行器可能会继续移动，从而导致调整失灵。而将pitch和roll设置为0，则可以保证四轴飞行器保持在稳定的位置，从而更容易进行微调。
					值得注意的是，在非微调模式下（TrimFlag为假），pitch和roll是根据通过传感器获取的飞行器的数据进行设置的，这样即可进行偏移、滚转等复杂的运动。
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
			
			/*发送飞控数据*/
			NRF24L01_TxPacket((u8*)&send);
		}
		
		/*发送遥感数据至匿名上位机*/
		// todo 考虑发送到串口
	}
}

#endif
