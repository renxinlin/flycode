#include "outterControl.h"


#include <math.h>
#include "pid.h"
#include "stdio.h"
#include "maths.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 位置PID控制代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
 *
 * 修改说明:
 * 版本V1.3 水平定点PID输出较大，所以在位置环输出设置0.1的系数，
	速率环输出设置0.15系数，从而增加PID的可调性。
********************************************************************************/

#define THRUST_BASE  		(65534/2)	/*基础油门值*/

#define PIDVX_OUTPUT_LIMIT	120.0f	//ROLL限幅	(单位°带0.15的系数)
#define PIDVY_OUTPUT_LIMIT	120.0f 	//PITCH限幅	(单位°带0.15的系数)
#define PIDVZ_OUTPUT_LIMIT	(40000)	/*PID VZ限幅*/

#define PIDX_OUTPUT_LIMIT	1200.0f	//X轴速度限幅(单位cm/s 带0.1的系数)
#define PIDY_OUTPUT_LIMIT	1200.0f	//Y轴速度限幅(单位cm/s 带0.1的系数)
#define PIDZ_OUTPUT_LIMIT	120.0f	//Z轴速度限幅(单位cm/s)


extern configParam_data configParam; // pid持久化的数据



pid_data pidVX;
pid_data pidVY;
pid_data pidVZ;

pid_data pidX;
pid_data pidY;
pid_data pidZ;

void positionControlInit(float velocityPidDt, float posPidDt)
{
	pidInit(&pidVX, 0, configParam.pidPos.vx, velocityPidDt);	/*vx PID初始化*/
	pidInit(&pidVY, 0, configParam.pidPos.vy, velocityPidDt);	/*vy PID初始化*/
	pidInit(&pidVZ, 0, configParam.pidPos.vz, velocityPidDt);	/*vz PID初始化*/
	pidSetOutputLimit(&pidVX, PIDVX_OUTPUT_LIMIT);		/* 输出限幅 */
	pidSetOutputLimit(&pidVY, PIDVY_OUTPUT_LIMIT);		/* 输出限幅 */
	pidSetOutputLimit(&pidVZ, PIDVZ_OUTPUT_LIMIT);		/* 输出限幅 */
	
	pidInit(&pidX, 0, configParam.pidPos.x, posPidDt);			/*x PID初始化*/
	pidInit(&pidY, 0, configParam.pidPos.y, posPidDt);			/*y PID初始化*/
	pidInit(&pidZ, 0, configParam.pidPos.z, posPidDt);			/*z PID初始化*/
	pidSetOutputLimit(&pidX, PIDX_OUTPUT_LIMIT);		/* 输出限幅 */
	pidSetOutputLimit(&pidY, PIDY_OUTPUT_LIMIT);		/* 输出限幅 */
	pidSetOutputLimit(&pidZ, PIDZ_OUTPUT_LIMIT);		/* 输出限幅 */
}



void positionController(float* thrust, attitude_data *attitude, expect_data *setpoint, const self_data *state, float dt)                                                
{	

	// 先控制外环
	if (setpoint->mode.z == modeAbs)
	{
	//	printf(" fly expect height %f, actual height is %f \r\n",setpoint->position.z, state->position.z);
		setpoint->velocity.z = pidUpdate(&pidZ, setpoint->position.z - state->position.z);
	}
	// 传感器是否更新state->velocity.z与state->position.z 
	// todo 后面看看速率环可能直接删掉，直接使用高度环
	// printf(" fly expect height %f, actual height is %f \r\n",setpoint->velocity.z, state->velocity.z);
	float thrustRaw = pidUpdate(&pidVZ, setpoint->velocity.z - state->velocity.z);
	*thrust = constrainf(thrustRaw + THRUST_BASE, THRUST_BASE, 60000);	/*油门限幅*/
}


void positionResetAllPID(void)
{
	pidReset(&pidVX);
	pidReset(&pidVY);
	pidReset(&pidVZ);

	pidReset(&pidX);
	pidReset(&pidY);
	pidReset(&pidZ);
}

void positionPIDwriteToConfigParam(void)
{
	configParam.pidPos.vx.kp  = pidVX.kp;
	configParam.pidPos.vx.ki  = pidVX.ki;
	configParam.pidPos.vx.kd  = pidVX.kd;
	
	configParam.pidPos.vy.kp  = pidVY.kp;
	configParam.pidPos.vy.ki  = pidVY.ki;
	configParam.pidPos.vy.kd  = pidVY.kd;
	
	configParam.pidPos.vz.kp  = pidVZ.kp;
	configParam.pidPos.vz.ki  = pidVZ.ki;
	configParam.pidPos.vz.kd  = pidVZ.kd;
	
	configParam.pidPos.x.kp  = pidX.kp;
	configParam.pidPos.x.ki  = pidX.ki;
	configParam.pidPos.x.kd  = pidX.kd;
	
	configParam.pidPos.y.kp  = pidY.kp;
	configParam.pidPos.y.ki  = pidY.ki;
	configParam.pidPos.y.kd  = pidY.kd;
	
	configParam.pidPos.z.kp  = pidZ.kp;
	configParam.pidPos.z.ki  = pidZ.ki;
	configParam.pidPos.z.kd  = pidZ.kd;
}






