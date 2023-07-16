#include "positionEstimate.h"

#define ABS(x) 		(((x) < 0) ? (-x) : (x))

static uint8_t isRstHeight ;	/*复位高度 1 复位*/
static uint8_t isRstAll = 1 ;		/*复位估测 1 复位*/

static float fusedHeight;			/*融合高度，起飞点为0*/
static float fusedHeightLpf = 0.f;	/*融合高度，低通*/
static float startBaroAsl = 0.f;	/*起飞点海拔*/



/*估测系统*/
static estimator_data estimator = 
{
	.vAccDeadband = 4.0f,
	.accBias[0] =  0.0f,
	.accBias[1] =  0.0f,
	.accBias[2] =  0.0f,
	.acc[0] = 0.0f,
	.acc[1] = 0.0f,
	.acc[2] = 0.0f,
	.vel[0] = 0.0f,
	.vel[1] = 0.0f,
	.vel[2] = 0.0f,
	.pos[0] = 0.0f,
	.pos[1] = 0.0f,
	.pos[2] = 0.0f,
};


void positionEstimate(sensor_data* sensorData, self_data* selfData, float dt) 
{	
	static float rangeLpf = 0.f;
	static float accLpf[3] = {0.f};		/*加速度低通*/	
	float weight = wBaro;

	float relateHight = sensorData->baro.asl - startBaroAsl;	/*气压相对高度*/
	
	
	fusedHeight = relateHight;	/*融合高度*/
	fusedHeightLpf += (fusedHeight - fusedHeightLpf) * 0.1f;	/*融合高度 低通*/
	
	if(isRstHeight)
	{	
		isRstHeight = 0;
		weight = 0.95f;		/*增加权重，快速调整*/	
		startBaroAsl = sensorData->baro.asl;
		estimator.pos[2] = fusedHeight;
	}
	else if(isRstAll)
	{
		isRstAll = 0;
		accLpf[2] = 0.f;	
		fusedHeight  = 0.f;
		fusedHeightLpf = 0.f;
		startBaroAsl = sensorData->baro.asl;
		estimator.vel[2] = 0.f;
		estimator.pos[2] = fusedHeight;
	}	
	
	
	acc_data accelBF;
	
	accelBF.x = sensorData->acc.x * GRAVITY_CMSS - estimator.accBias[0];
	accelBF.y = sensorData->acc.y * GRAVITY_CMSS - estimator.accBias[1];
	accelBF.z = sensorData->acc.z * GRAVITY_CMSS - estimator.accBias[2];	
	
	/* Rotate vector to Earth frame - from Forward-Right-Down to North-East-Up*/
	imuTransformVectorBodyToEarth(&accelBF);	
	
	estimator.acc[0] = applyDeadbandf(accelBF.x, estimator.vAccDeadband);/*去除死区的加速度*/
	estimator.acc[1] = applyDeadbandf(accelBF.y, estimator.vAccDeadband);/*去除死区的加速度*/
	estimator.acc[2] = applyDeadbandf(accelBF.z, estimator.vAccDeadband);/*去除死区的加速度*/
	
	for(u8 i=0; i<3; i++)
		accLpf[i] += (estimator.acc[i] - accLpf[i]) * 0.1f;	/*加速度低通*/
		
	uint8_t isKeyFlightLand = commanderBits.keyFlight||commanderBits.keyLand;	/*定高飞或者降落状态*/
	
	if(isKeyFlightLand)		/*定高飞或者降落状态*/
	{
		selfData->acc.x = constrainf(accLpf[0], -ACC_LIMIT, ACC_LIMIT);	/*加速度限幅*/
		selfData->acc.y = constrainf(accLpf[1], -ACC_LIMIT, ACC_LIMIT);	/*加速度限幅*/
		selfData->acc.z = constrainf(accLpf[2], -ACC_LIMIT, ACC_LIMIT);	/*加速度限幅*/
	}else
	{
		selfData->acc.x = constrainf(estimator.acc[0], -ACC_LIMIT_MAX, ACC_LIMIT_MAX);	/*最大加速度限幅*/
		selfData->acc.y = constrainf(estimator.acc[1], -ACC_LIMIT_MAX, ACC_LIMIT_MAX);	/*最大加速度限幅*/
		selfData->acc.z = constrainf(estimator.acc[2], -ACC_LIMIT_MAX, ACC_LIMIT_MAX);	/*最大加速度限幅*/
	}		

	
	float errPosZ = fusedHeight - estimator.pos[2];
	
	/* 速度与位置预估: Z-axis */
	inavFilterPredict(2, dt, estimator.acc[2]);
	/* 速度位置校正: Z-axis */
	inavFilterCorrectPos(2, dt, errPosZ, weight);	

	 
	/*加速度偏置校正*/
	acc_data accelBiasCorr = { 0, 0, 0};
	
	accelBiasCorr.z -= errPosZ  * sq(wBaro);
	float accelBiasCorrMagnitudeSq = sq(accelBiasCorr.x) + sq(accelBiasCorr.y) + sq(accelBiasCorr.z);
	// 加速度纠正
	if (accelBiasCorrMagnitudeSq < sq(INAV_ACC_BIAS_ACCEPTANCE_VALUE)) 
	{
		// 气压计在地球方向的误差纠正机体方向的加速度
		imuTransformVectorEarthToBody(&accelBiasCorr);

		/* Correct accel bias */
		estimator.accBias[0] += accelBiasCorr.x * wAccBias * dt;
		estimator.accBias[1] += accelBiasCorr.y * wAccBias * dt;
		estimator.accBias[2] += accelBiasCorr.z * wAccBias * dt;
	}	

	// 限幅处理
	if(isKeyFlightLand)		/*定高飞或者降落状态*/
	{
		selfData->velocity.z = constrainf(estimator.vel[2], -VELOCITY_LIMIT, VELOCITY_LIMIT);	/*速度限幅 VELOCITY_LIMIT*/
	}else
	{
		selfData->velocity.z = constrainf(estimator.vel[2], -VELOCITY_LIMIT_MAX, VELOCITY_LIMIT_MAX);	/*最大速度限幅 VELOCITY_LIMIT_MAX*/
	}
	// 设置四轴自身姿态
	selfData->position.z = estimator.pos[2];	
}


/* Inertial filter, implementation taken from PX4 implementation by Anton Babushkin <rk3dov@gmail.com> */
static void inavFilterPredict(int axis, float dt, float acc)
{
    estimator.pos[axis] += estimator.vel[axis] * dt + acc * dt * dt / 2.0f;
    estimator.vel[axis] += acc * dt;
}
/*位置校正*/
static void inavFilterCorrectPos(int axis, float dt, float e, float w)
{
    float ewdt = e * w * dt;
    estimator.pos[axis] += ewdt;
    estimator.vel[axis] += w * ewdt;
}

void estRstAll(void)
{
	isRstAll = 1;
}


 