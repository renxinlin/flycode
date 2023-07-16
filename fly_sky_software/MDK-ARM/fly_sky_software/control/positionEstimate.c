#include "positionEstimate.h"

#define ABS(x) 		(((x) < 0) ? (-x) : (x))

static uint8_t isRstHeight ;	/*��λ�߶� 1 ��λ*/
static uint8_t isRstAll = 1 ;		/*��λ���� 1 ��λ*/

static float fusedHeight;			/*�ںϸ߶ȣ���ɵ�Ϊ0*/
static float fusedHeightLpf = 0.f;	/*�ںϸ߶ȣ���ͨ*/
static float startBaroAsl = 0.f;	/*��ɵ㺣��*/



/*����ϵͳ*/
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
	static float accLpf[3] = {0.f};		/*���ٶȵ�ͨ*/	
	float weight = wBaro;

	float relateHight = sensorData->baro.asl - startBaroAsl;	/*��ѹ��Ը߶�*/
	
	
	fusedHeight = relateHight;	/*�ںϸ߶�*/
	fusedHeightLpf += (fusedHeight - fusedHeightLpf) * 0.1f;	/*�ںϸ߶� ��ͨ*/
	
	if(isRstHeight)
	{	
		isRstHeight = 0;
		weight = 0.95f;		/*����Ȩ�أ����ٵ���*/	
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
	
	estimator.acc[0] = applyDeadbandf(accelBF.x, estimator.vAccDeadband);/*ȥ�������ļ��ٶ�*/
	estimator.acc[1] = applyDeadbandf(accelBF.y, estimator.vAccDeadband);/*ȥ�������ļ��ٶ�*/
	estimator.acc[2] = applyDeadbandf(accelBF.z, estimator.vAccDeadband);/*ȥ�������ļ��ٶ�*/
	
	for(u8 i=0; i<3; i++)
		accLpf[i] += (estimator.acc[i] - accLpf[i]) * 0.1f;	/*���ٶȵ�ͨ*/
		
	uint8_t isKeyFlightLand = commanderBits.keyFlight||commanderBits.keyLand;	/*���߷ɻ��߽���״̬*/
	
	if(isKeyFlightLand)		/*���߷ɻ��߽���״̬*/
	{
		selfData->acc.x = constrainf(accLpf[0], -ACC_LIMIT, ACC_LIMIT);	/*���ٶ��޷�*/
		selfData->acc.y = constrainf(accLpf[1], -ACC_LIMIT, ACC_LIMIT);	/*���ٶ��޷�*/
		selfData->acc.z = constrainf(accLpf[2], -ACC_LIMIT, ACC_LIMIT);	/*���ٶ��޷�*/
	}else
	{
		selfData->acc.x = constrainf(estimator.acc[0], -ACC_LIMIT_MAX, ACC_LIMIT_MAX);	/*�����ٶ��޷�*/
		selfData->acc.y = constrainf(estimator.acc[1], -ACC_LIMIT_MAX, ACC_LIMIT_MAX);	/*�����ٶ��޷�*/
		selfData->acc.z = constrainf(estimator.acc[2], -ACC_LIMIT_MAX, ACC_LIMIT_MAX);	/*�����ٶ��޷�*/
	}		

	
	float errPosZ = fusedHeight - estimator.pos[2];
	
	/* �ٶ���λ��Ԥ��: Z-axis */
	inavFilterPredict(2, dt, estimator.acc[2]);
	/* �ٶ�λ��У��: Z-axis */
	inavFilterCorrectPos(2, dt, errPosZ, weight);	

	 
	/*���ٶ�ƫ��У��*/
	acc_data accelBiasCorr = { 0, 0, 0};
	
	accelBiasCorr.z -= errPosZ  * sq(wBaro);
	float accelBiasCorrMagnitudeSq = sq(accelBiasCorr.x) + sq(accelBiasCorr.y) + sq(accelBiasCorr.z);
	// ���ٶȾ���
	if (accelBiasCorrMagnitudeSq < sq(INAV_ACC_BIAS_ACCEPTANCE_VALUE)) 
	{
		// ��ѹ���ڵ���������������巽��ļ��ٶ�
		imuTransformVectorEarthToBody(&accelBiasCorr);

		/* Correct accel bias */
		estimator.accBias[0] += accelBiasCorr.x * wAccBias * dt;
		estimator.accBias[1] += accelBiasCorr.y * wAccBias * dt;
		estimator.accBias[2] += accelBiasCorr.z * wAccBias * dt;
	}	

	// �޷�����
	if(isKeyFlightLand)		/*���߷ɻ��߽���״̬*/
	{
		selfData->velocity.z = constrainf(estimator.vel[2], -VELOCITY_LIMIT, VELOCITY_LIMIT);	/*�ٶ��޷� VELOCITY_LIMIT*/
	}else
	{
		selfData->velocity.z = constrainf(estimator.vel[2], -VELOCITY_LIMIT_MAX, VELOCITY_LIMIT_MAX);	/*����ٶ��޷� VELOCITY_LIMIT_MAX*/
	}
	// ��������������̬
	selfData->position.z = estimator.pos[2];	
}


/* Inertial filter, implementation taken from PX4 implementation by Anton Babushkin <rk3dov@gmail.com> */
static void inavFilterPredict(int axis, float dt, float acc)
{
    estimator.pos[axis] += estimator.vel[axis] * dt + acc * dt * dt / 2.0f;
    estimator.vel[axis] += acc * dt;
}
/*λ��У��*/
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


 