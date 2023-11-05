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

#define MAIN_LOOP_RATE 	RATE_1000_HZ   // 确认主循环的hz
#define MAIN_LOOP_DT	(u32)(1000/MAIN_LOOP_RATE)	/*单位ms*/

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
static float actualThrust;  // 实际油门
static attitude_data attitudeDesired; // 期望姿态角
static attitude_data rateDesired;  // 期望角速度

void stateControlInit(void)
{
	 attitudeControlInit(RATE_PID_DT, ANGEL_PID_DT); /*初始化姿态PID*/	
	 positionControlInit(VELOCITY_PID_DT, POSITION_PID_DT); /*初始化位置PID*/
}


 
void stateControl(control_data *control, sensor_data *sensors, self_data *self, expect_data *setpoint, const uint32_t tick)
{	
	// 最终的pid控制
	// 传感器数据
	// 四轴自身数据
	// printf(" self data is %f ,%f,%f",setpoint->attitude.x,setpoint->attitude.y,setpoint->attitude.z);
	// 期望的命令数据
	
	if (tick >=1)
	{

		if (setpoint->mode.z != modeDisable)
		{
			// 高度与速度环（外环）
			positionController(&actualThrust, &attitudeDesired, setpoint, self, POSITION_PID_DT);
		}
		if (setpoint->mode.z == modeDisable)
		{
			actualThrust = setpoint->thrust;
		}
		/**
				位置环不参与直接使用遥控器值
	
		*/
		if (setpoint->mode.x == modeDisable || setpoint->mode.y == modeDisable) 
		{
			attitudeDesired.x = setpoint->attitude.x;
			attitudeDesired.y = setpoint->attitude.y;
		}
			
		attitudeDesired.x += configParam.trimR;	//叠加微调值
		attitudeDesired.y += configParam.trimP;		
		// yaw 位置环不参与直接使用遥控器值
		attitudeDesired.z=setpoint->attitude.z;
		attitudeAnglePID(&self->attitude, &attitudeDesired, &rateDesired);
	}

	  //角速度环（内环）
		// 角速度 期望内环准备开始调试
		attitudeRatePID(&sensors->gyro, &rateDesired, control);
		control->thrust = actualThrust;	
}


static uint8_t motorSetEnable = 0;
static motorPwm_data motorPWM;
static motorPwm_data motorPWMSet={0, 0, 0, 0};





void powerControl(control_data *control)	/*功率输出控制*/
{
	int16_t r = control->roll / 2.0f; // [-65535,65535]=》【-32767,32767】
	int16_t p = control->pitch / 2.0f;
	// 电机设置依据调整
	/**
	800.000000,196,1274,270    

	2 3
	1 4                                    翅膀正
	    -x                                 ^  y
	                               开关    |
	 +y                      机头----------------x--->      
	                                       |
	
	      [高 
	       低-45]
	
		逆时针正，顺时针负数
	  
yaw	
	正<--
		    |
	 负---
	
	左转13大24小
	
	
	俯仰
	
 		公式与pid输出正负号密切相关
	*/
	motorPWM.m1 = limitThrust(control->thrust + r - p + control->yaw);
	
	motorPWM.m2 = limitThrust(control->thrust - r - p - control->yaw);
	
	motorPWM.m3 = limitThrust(control->thrust - r + p + control->yaw);
	
	motorPWM.m4 = limitThrust(control->thrust + r + p - control->yaw);		


	 //  float thrust = ((float)motorPWM.m1 / 65536.0f) * 60;
			// 系数-0.0006239f和0.088f用于定义推力和电压之间的关系
			//float volts = -0.0006239f * thrust * thrust + 0.088f * thrust;
			//float percentage = volts / voltage;
			//percentage = percentage > 1.0f ? 1.0f : percentage;
			//motorPWM.m1 = percentage * UINT16_MAX;
			 
	if (motorSetEnable)
	{
		motorPWM = motorPWMSet;
	}
	//printf(" m1 = %d m2 = %d m3 = %d m4 = %d ",motorPWM.m1,motorPWM.m2,motorPWM.m3,motorPWM.m4);
	motorsSetRatio(MOTOR_M1, motorPWM.m1);	/*控制电机输出百分比*/
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
		float thrust = ((float)ithrust / 65536.0f) * 60;// 油门从每秒转换为每分钟
		float volts = -0.0006239f * thrust * thrust + 0.088f * thrust; //计算推力
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
    uint16_t result = val / 65.535;  // 进行除法运算

    if (result > 1000) {   // 如果结果大于1000，则输出1000
        return 1000;
    } else if (result < 0) {   // 如果结果小于0，则输出0
        return 0;
    } else {
        return result;    // 其他情况输出原来的结果
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

