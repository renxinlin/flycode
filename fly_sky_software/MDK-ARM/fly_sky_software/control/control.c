#include "control.h"
#include "main.h"


#define RATE_250_HZ 	250 
#define RATE_500_HZ 	500 
#define RATE_1000_HZ 	1000 
 

#define RATE_PID_RATE			(RATE_500_HZ)
#define RATE_PID_DT				(1.0/RATE_500_HZ)

#define ANGEL_PID_RATE			(RATE_250_HZ)
#define ANGEL_PID_DT			(1.0/RATE_250_HZ)

#define VELOCITY_PID_RATE			(RATE_250_HZ)
#define VELOCITY_PID_DT			(1.0/RATE_250_HZ)

#define POSITION_PID_RATE			(RATE_250_HZ)
#define POSITION_PID_DT			(1.0/RATE_250_HZ)

#define MAIN_LOOP_RATE 	RATE_1000_HZ   // ȷ����ѭ����hz
#define MAIN_LOOP_DT	(u32)(1000/MAIN_LOOP_RATE)	/*��λms*/

#define RATE_DO_EXECUTE(RATE_HZ, TICK) ((TICK % (MAIN_LOOP_RATE / RATE_HZ)) == 0)


#define NBR_OF_MOTORS 	4
#define MOTOR_M1  		0
#define MOTOR_M2  		1
#define MOTOR_M3  		2
#define MOTOR_M4  		3
#define MOTORS_PWM_BITS           	16
static uint8_t isInit = 1;
uint32_t motor_ratios[] = {0, 0, 0, 0};
static const uint32_t MOTORS[] = { MOTOR_M1, MOTOR_M2, MOTOR_M3, MOTOR_M4 };

void motorsSetRatio(uint32_t id, uint16_t ithrust);
uint16_t limitThrust(int value);
 static uint16_t ratioToCCRx(uint16_t val);
static float actualThrust;  // ʵ������
static attitude_data attitudeDesired; // ������̬��
static attitude_data rateDesired;  // �������ٶ�

void stateControlInit(void)
{
	 attitudeControlInit(RATE_PID_DT, ANGEL_PID_DT); /*��ʼ����̬PID*/	
	 positionControlInit(VELOCITY_PID_DT, POSITION_PID_DT); /*��ʼ��λ��PID*/
}


 
void stateControl(control_data *control, sensor_data *sensors, self_data *self, expect_data *setpoint, const uint32_t tick)
{	
	// ���յ�pid����
	// ����������
	// ������������
	// printf(" self data is %f ,%f,%f",setpoint->attitude.x,setpoint->attitude.y,setpoint->attitude.z);
	// ��������������
	
	if (tick >=1)
	{

		if (setpoint->mode.z != modeDisable)
		{
			// �߶����ٶȻ����⻷��
			positionController(&actualThrust, &attitudeDesired, setpoint, self, POSITION_PID_DT);
		}
		if (setpoint->mode.z == modeDisable)
		{
			actualThrust = setpoint->thrust;
		}
		/**
				λ�û�������ֱ��ʹ��ң����ֵ
	
		*/
		if (setpoint->mode.x == modeDisable || setpoint->mode.y == modeDisable) 
		{
			attitudeDesired.x = setpoint->attitude.x;
			attitudeDesired.y = setpoint->attitude.y;
		}
			
		attitudeDesired.x += configParam.trimR;	//����΢��ֵ
		attitudeDesired.y += configParam.trimP;		
		// yaw λ�û�������ֱ��ʹ��ң����ֵ
		attitudeDesired.z=setpoint->attitude.z;
		attitudeAnglePID(&self->attitude, &attitudeDesired, &rateDesired);
	}

	  //���ٶȻ����ڻ���
		// ���ٶ� �����ڻ�׼����ʼ����
		attitudeRatePID(&sensors->gyro, &rateDesired, control);
		control->thrust = actualThrust;	
}


static uint8_t motorSetEnable = 0;
static motorPwm_data motorPWM;
static motorPwm_data motorPWMSet={0, 0, 0, 0};





void powerControl(control_data *control)	/*�����������*/
{
	int16_t r = control->roll / 2.0f; // [-65535,65535]=����-32767,32767��
	int16_t p = control->pitch / 2.0f;
	// ����������ݵ���
	/**
	800.000000,196,1274,270    

	2 3
	1 4                                    �����
	    -x                                 ^  y
	                               ����    |
	 +y                      ��ͷ----------------x--->      
	                                       |
	
	      [�� 
	       ��-45]
	
		��ʱ������˳ʱ�븺��
	  
yaw	
	��<--
		    |
	 ��---
	
	��ת13��24С
	
	
	����
	
 		��ʽ��pid����������������
	*/
	motorPWM.m1 = limitThrust(control->thrust + r - p + control->yaw);
	
	motorPWM.m2 = limitThrust(control->thrust - r - p - control->yaw);
	
	motorPWM.m3 = limitThrust(control->thrust - r + p + control->yaw);
	
	motorPWM.m4 = limitThrust(control->thrust + r + p - control->yaw);		


	 //  float thrust = ((float)motorPWM.m1 / 65536.0f) * 60;
			// ϵ��-0.0006239f��0.088f���ڶ��������͵�ѹ֮��Ĺ�ϵ
			//float volts = -0.0006239f * thrust * thrust + 0.088f * thrust;
			//float percentage = volts / voltage;
			//percentage = percentage > 1.0f ? 1.0f : percentage;
			//motorPWM.m1 = percentage * UINT16_MAX;
			 
	if (motorSetEnable)
	{
		motorPWM = motorPWMSet;
	}
	//printf(" m1 = %d m2 = %d m3 = %d m4 = %d ",motorPWM.m1,motorPWM.m2,motorPWM.m3,motorPWM.m4);
	motorsSetRatio(MOTOR_M1, motorPWM.m1);	/*���Ƶ������ٷֱ�*/
	motorsSetRatio(MOTOR_M2, motorPWM.m2);
	motorsSetRatio(MOTOR_M3, motorPWM.m3);
	motorsSetRatio(MOTOR_M4, motorPWM.m4);
}


uint16_t limitThrust(int value)
{
	if(value > UINT16_MAX)
	{
		value = UINT16_MAX;
	}
	else if(value < 0)
	{
		value = 0;
	}

	return (uint16_t)value;
}

void motorsSetRatio(uint32_t id, uint16_t ithrust)
{
	if (isInit) 
	{
		uint16_t ratio=ithrust;
		/*
		float thrust = ((float)ithrust / 65536.0f) * 60;// ���Ŵ�ÿ��ת��Ϊÿ����
		float volts = -0.0006239f * thrust * thrust + 0.088f * thrust; //��������
		float supply_voltage = pmGetBatteryVoltage();
		float percentage = volts / supply_voltage;
		percentage = percentage > 1.0f ? 1.0f : percentage;
		ratio = percentage * UINT16_MAX;
		motor_ratios[id] = ratio;*/
		switch(id)
		{
			case 0:		/*MOTOR_M1*/
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, ratioToCCRx(ratio));				break;
				break;
			case 1:		/*MOTOR_M2*/
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, ratioToCCRx(ratio));				break;
			case 2:		/*MOTOR_M3*/
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ratioToCCRx(ratio));				break;
				break;
			case 3:		/*MOTOR_M4*/	
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, ratioToCCRx(ratio));				break;
				break;
			default: break;
		}	
	}
}

static uint16_t ratioToCCRx(uint16_t val)
{
    uint16_t result = val / 65.535;  // ���г�������

    if (result > 1000) {   // ����������1000�������1000
        return 1000;
    } else if (result < 0) {   // ������С��0�������0
        return 0;
    } else {
        return result;    // ����������ԭ���Ľ��
    }
}
void setMotorPWM(uint8_t enable, uint32_t m1_set, uint32_t m2_set, uint32_t m3_set, uint32_t m4_set)
{
	motorSetEnable = enable;
	motorPWMSet.m1 = m1_set;
	motorPWMSet.m2 = m2_set;
	motorPWMSet.m3 = m3_set;	
	motorPWMSet.m4 = m4_set;
}

