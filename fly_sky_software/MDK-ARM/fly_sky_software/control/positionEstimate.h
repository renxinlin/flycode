#ifndef __POSITION_H
#define __POSITION_H

#include "imu.h"
#include "data.h"
#include "math.h"
#include "maths.h"


typedef struct
{
	float vAccDeadband; /* ���ٶ����� */
	float accBias[3];	/* ���ٶ� ƫ��(cm/s/s)*/
	float acc[3];		/* ������ٶ� ��λ(cm/s/s)*/
	float vel[3];		/* �����ٶ� ��λ(cm/s)*/
	float pos[3]; 		/* ����λ�� ��λ(cm)*/
} estimator_data;
/**

��ѹ�Ƶ�Ȩ����Ҫ���ݾ���������е�����һ����˵����ѹ�Ƶ�Ȩ��Ӧ����0.1��0.3֮�䡣����������ڽϸߵĸ߶��Ϸ��У���ѹ�Ƶ�Ȩ�ؿ����ʵ����ӣ�����ڵ͸߶��Ϸ��У������ʵ�������ѹ�Ƶ�Ȩ�ء�

ȷ����ѹ�Ƶ�Ȩ����Ҫ�����������أ�

1. �������ĸ߶ȷ�Χ������������ڽϸߵĸ߶��Ϸ��У���ѹ�Ƶ�Ȩ�ؿ����ʵ����ӣ�����ڵ͸߶��Ϸ��У������ʵ�������ѹ�Ƶ�Ȩ�ء�

2. ��ѹ�Ƶľ��ȣ������ѹ�Ƶľ��Ƚϸߣ������ʵ�������Ȩ�ء�

3. �����������ľ��ȣ���������������ľ��Ƚϸߣ������ʵ�������ѹ�Ƶ�Ȩ�ء�

4. �������Ķ�̬���ԣ�����������Ķ�̬���Խϴ󣬿����ʵ�������ѹ�Ƶ�Ȩ�أ��Լ�����Է�������Ӱ�졣

�ۺϿ����������أ����Ը���ʵ��������е������Դﵽ��ѵķ���Ч����
*/

#define ACC_LIMIT			(1000.f)/*���ٶ��޷� ��λcm/s/s*/
#define ACC_LIMIT_MAX		(1800.f)/*�����ٶ��޷� ��λcm/s/s*/
#define VELOCITY_LIMIT		(130.f)	/*�ٶ��޷� ��λcm/s*/
#define VELOCITY_LIMIT_MAX	(500.f)	/*����ٶ��޷� ��λcm/s*/

#define GRAVITY_CMSS 		(980.f)	/*�������ٶ� ��λcm/s/s*/
#define INAV_ACC_BIAS_ACCEPTANCE_VALUE	(GRAVITY_CMSS * 0.25f)   // Max accepted bias correction of 0.25G - unlikely we are going to be that much off anyway


static float wBaro = 0.75f;			/*��ѹУ��Ȩ��*/
static float wAccBias = 0.001f;		/*���ٶ�У��Ȩ��*/

void positionEstimate(sensor_data* sensorData, self_data* selfData, float dt);




static void inavFilterPredict(int axis, float dt, float acc);
/*λ��У��*/
static void inavFilterCorrectPos(int axis, float dt, float e, float w);
void estRstAll(void);


#endif 
