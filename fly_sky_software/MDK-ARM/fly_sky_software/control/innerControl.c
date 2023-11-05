#include "innerControl.h"
#include "stdio.h"

extern configParam_data configParam; // pid持久化的数据

/*角度环积分限幅*/
#define PID_ANGLE_ROLL_INTEGRATION_LIMIT    30.0
#define PID_ANGLE_PITCH_INTEGRATION_LIMIT   30.0
#define PID_ANGLE_YAW_INTEGRATION_LIMIT     180.0

/*角速度环积分限幅*/
#define PID_RATE_ROLL_INTEGRATION_LIMIT		500.0
#define PID_RATE_PITCH_INTEGRATION_LIMIT	500.0
#define PID_RATE_YAW_INTEGRATION_LIMIT		50.0

 pid_data pidAngleRoll;
 pid_data pidAnglePitch;
 pid_data pidAngleYaw;

 pid_data pidRateRoll;
 pid_data pidRatePitch;
 pid_data pidRateYaw;
 
 
 
 
static inline int16_t pidOutLimit(float in)
{
	// 32767
	if (in > INT16_MAX)
		return INT16_MAX;
	else if (in < -INT16_MAX)
		return -INT16_MAX;
	else
		return (int16_t)in;
}

void attitudeControlInit(float ratePidDt, float anglePidDt)
{
	pidInit(&pidAngleRoll, 0, configParam.pidAngle.roll, anglePidDt);			/*roll  角度PID初始化*/
	pidInit(&pidAnglePitch, 0, configParam.pidAngle.pitch, anglePidDt);			/*pitch 角度PID初始化*/
	pidInit(&pidAngleYaw, 0, configParam.pidAngle.yaw, anglePidDt);				/*yaw   角度PID初始化*/
	pidSetIntegralLimit(&pidAngleRoll, PID_ANGLE_ROLL_INTEGRATION_LIMIT);		/*roll  角度积分限幅设置*/
	pidSetIntegralLimit(&pidAnglePitch, PID_ANGLE_PITCH_INTEGRATION_LIMIT);		/*pitch 角度积分限幅设置*/
	pidSetIntegralLimit(&pidAngleYaw, PID_ANGLE_YAW_INTEGRATION_LIMIT);			/*yaw   角度积分限幅设置*/
	
	pidInit(&pidRateRoll, 0, configParam.pidRate.roll, ratePidDt);				/*roll  角速度PID初始化*/
	pidInit(&pidRatePitch, 0, configParam.pidRate.pitch, ratePidDt);			/*pitch 角速度PID初始化*/
	pidInit(&pidRateYaw, 0, configParam.pidRate.yaw, ratePidDt);				/*yaw   角速度PID初始化*/
	pidSetIntegralLimit(&pidRateRoll, PID_RATE_ROLL_INTEGRATION_LIMIT);			/*roll  角速度积分限幅设置*/
	pidSetIntegralLimit(&pidRatePitch, PID_RATE_PITCH_INTEGRATION_LIMIT);		/*pitch 角速度积分限幅设置*/
	pidSetIntegralLimit(&pidRateYaw, PID_RATE_YAW_INTEGRATION_LIMIT);			/*yaw   角速度积分限幅设置*/
}

 

void attitudeRatePID(acc_data *actualRate,acc_data *desiredRate,control_data *output)	/* 角速度环PID */
{
	output->roll = pidOutLimit(pidUpdate(&pidRateRoll, desiredRate->x - actualRate->x));
	output->pitch = pidOutLimit(pidUpdate(&pidRatePitch, desiredRate->y - actualRate->y));
	output->yaw = pidOutLimit(pidUpdate(&pidRateYaw, desiredRate->z - actualRate->z));

	/*
	printf("================================================================\r\n");
	printf("pid out is %f\r\n",pidRateRoll.out);
	printf("outP %f\r\n",pidRateRoll.outP);
	printf("outI %f\r\n",pidRateRoll.outI);
	printf("outD %f\r\n",pidRateRoll.outD);
	printf("dt %f\r\n",pidRateRoll.dt);
	printf("iLimit %f\r\n",pidRateRoll.iLimit);
	printf("integ %f\r\n",pidRateRoll.integ);
	printf("error %f\r\n",pidRateRoll.error);
	printf("kd %f\r\n",pidRateRoll.kd);
	printf("ki %f\r\n",pidRateRoll.ki);
	printf("kp %f\r\n",pidRateRoll.kp);
	printf("================================================================\r\n");
	printf("pid pitch out is %f\r\n",pidRatePitch.out);
	printf("outP %f\r\n",pidRatePitch.outP);
	printf("outI %f\r\n",pidRatePitch.outI);
	printf("outD %f\r\n",pidRatePitch.outD);
	printf("dt %f\r\n",pidRatePitch.dt);
	printf("iLimit %f\r\n",pidRatePitch.iLimit);
	printf("integ %f\r\n",pidRatePitch.integ);
	printf("error %f\r\n",pidRatePitch.error);
	printf("kd %f\r\n",pidRatePitch.kd);
	printf("ki %f\r\n",pidRatePitch.ki);
	printf("kp %f\r\n",pidRatePitch.kp);
	*/
}

void attitudeAnglePID(attitude_data *actualAngle,attitude_data *desiredAngle,acc_data *outDesiredRate)	/* 角度环PID */
{
	outDesiredRate->x = pidUpdate(&pidAngleRoll, desiredAngle->x - actualAngle->x);
	outDesiredRate->y = pidUpdate(&pidAnglePitch, desiredAngle->y - actualAngle->y);

	float yawError = desiredAngle->z - actualAngle->z ;
	if (yawError > 180.0f) 
		yawError -= 360.0f;
	else if (yawError < -180.0) 
		yawError += 360.0f;
	outDesiredRate->z = pidUpdate(&pidAngleYaw, yawError);
}

void attitudeControllerResetRollAttitudePID(void)
{
    pidReset(&pidAngleRoll);
}

void attitudeControllerResetPitchAttitudePID(void)
{
    pidReset(&pidAnglePitch);
}

void attitudeResetAllPID(void)	/*复位PID*/
{
	pidReset(&pidAngleRoll);
	pidReset(&pidAnglePitch);
	pidReset(&pidAngleYaw);
	pidReset(&pidRateRoll);
	pidReset(&pidRatePitch);
	pidReset(&pidRateYaw);
}

void attitudePIDwriteToConfigParam(void)
{
	configParam.pidAngle.roll.kp = pidAngleRoll.kp;
	configParam.pidAngle.roll.ki = pidAngleRoll.ki;
	configParam.pidAngle.roll.kd = pidAngleRoll.kd;
	
	configParam.pidAngle.pitch.kp = pidAnglePitch.kp;
	configParam.pidAngle.pitch.ki = pidAnglePitch.ki;
	configParam.pidAngle.pitch.kd = pidAnglePitch.kd;
	
	configParam.pidAngle.yaw.kp = pidAngleYaw.kp;
	configParam.pidAngle.yaw.ki = pidAngleYaw.ki;
	configParam.pidAngle.yaw.kd = pidAngleYaw.kd;
	
	configParam.pidRate.roll.kp = pidRateRoll.kp;
	configParam.pidRate.roll.ki = pidRateRoll.ki;
	configParam.pidRate.roll.kd = pidRateRoll.kd;
	
	configParam.pidRate.pitch.kp = pidRatePitch.kp;
	configParam.pidRate.pitch.ki = pidRatePitch.ki;
	configParam.pidRate.pitch.kd = pidRatePitch.kd;
	
	configParam.pidRate.yaw.kp = pidRateYaw.kp;
	configParam.pidRate.yaw.ki = pidRateYaw.ki;
	configParam.pidRate.yaw.kd = pidRateYaw.kd;
}
