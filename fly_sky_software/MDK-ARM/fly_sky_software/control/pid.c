#include "pid.h"

void pidInit(pid_data* pid, const float desired, const pidInit_data  pidParam, const float dt)
{
	pid->error     = 0;
	pid->prevError = 0;
	pid->integ     = 0;
	pid->deriv     = 0;
	pid->desired = desired;
	pid->kp = pidParam.kp;
	pid->ki = pidParam.ki;
	pid->kd = pidParam.kd;
	pid->iLimit = DEFAULT_PID_INTEGRATION_LIMIT;
	pid->outputLimit = DEFAULT_PID_OUTPUT_LIMIT;
	pid->dt = dt;
}

float pidUpdate(pid_data* pid, const float error)
{
	float output;

	pid->error = error;   

	pid->integ += pid->error * pid->dt;
	
	//积分限幅
	if (pid->integ > pid->iLimit)
	{
		pid->integ = pid->iLimit;
	}
	else if (pid->integ < -pid->iLimit)
	{
		pid->integ = -pid->iLimit;
	}

	pid->deriv = (pid->error - pid->prevError) / pid->dt;

	pid->outP = pid->kp * pid->error;
	pid->outI = pid->ki * pid->integ;
	pid->outD = pid->kd * pid->deriv;

	output = pid->outP + pid->outI + pid->outD;
	
	//输出限幅
	if (pid->outputLimit != 0)
	{
		if (output > pid->outputLimit)
			output = pid->outputLimit;
		else if (output < -pid->outputLimit)
			output = -pid->outputLimit;
	}
	
	pid->prevError = pid->error;

	pid->out = output;
	return output;
}

void pidSetIntegralLimit(pid_data* pid, const float limit) 
{
    pid->iLimit = limit;
}

void pidSetOutputLimit(pid_data* pid, const float limit) 
{
	pid->outputLimit = limit;
}

void pidSetError(pid_data* pid, const float error)
{
	pid->error = error;
}

void pidSetDesired(pid_data* pid, const float desired)
{
	pid->desired = desired;
}

float pidGetDesired(pid_data* pid)
{
	return pid->desired;
}

void pidSetKp(pid_data* pid, const float kp)
{
	pid->kp = kp;
}

void pidSetKi(pid_data* pid, const float ki)
{
	pid->ki = ki;
}

void pidSetKd(pid_data* pid, const float kd)
{
	pid->kd = kd;
}

void pidSetDt(pid_data* pid, const float dt) 
{
    pid->dt = dt;
}

void pidReset(pid_data* pid)
{
	pid->error     = 0;
	pid->prevError = 0;
	pid->integ     = 0;
	pid->deriv     = 0;
}