#ifndef __IMU_H
#define __IMU_H

#include "data.h"
#include "imu.h"
#include "maths.h"
#include "stdio.h"
#include "math.h"
#define ACCZ_SAMPLE		350

// �˴��ǰ�װ��Ԫ������׼���̶����pid���֣���������̬pid����
float Kp = 20.f;		/*��Ҫʵ�ʵ���Ӳ����ú��ʵ�p ����:18 <p<30 */
float Ki = 3.f;		/*��Ҫʵ�ʵ���Ӳ����ú��ʵ�p  1<i<5*/

float exInt = 0.0f;
float eyInt = 0.0f;
float ezInt = 0.0f;		/*��������ۼ�*/

static float q0 = 1.0f;	/*��Ԫ��*/
static float q1 = 0.0f;
static float q2 = 0.0f;
static float q3 = 0.0f;	
static float rMat[3][3];/*���Ҿ�����ת����*/


static float maxError = 0.f;		/*������*/
uint8_t isGravityCalibrated = 0;	/*�Ƿ�УУ׼��� 0���*/
static float baseAcc[3] = {0.f,0.f,1.0f};	/*��̬���ٶ�*/

void imucalcuate(void);
void calBaseAcc(float* acc);
#endif


/*������ת����*/
void imuComputeRotationMatrix(void)
{
		// ��Ԫ��ת���Ҿ���ʽ
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;

    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q3 = q2 * q3;
		// rMat[0][0] = q0*q0+q1*q1-q2*q2-q3*q3;
    rMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
    rMat[0][1] = 2.0f * (q1q2 + -q0q3);
    rMat[0][2] = 2.0f * (q1q3 - -q0q2);

    rMat[1][0] = 2.0f * (q1q2 - -q0q3);
    rMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
    rMat[1][2] = 2.0f * (q2q3 + -q0q1);

    rMat[2][0] = 2.0f * (q1q3 + -q0q2);
    rMat[2][1] = 2.0f * (q2q3 - -q0q1);
    rMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}

void imuUpdate(acc_data acc, gyro_data gyro, self_data *state , float dt)	/*�����ں� �����˲�*/
{
	float normalise;
	float ex, ey, ez;
	float halfT = 0.5f * dt;
	float accBuf[3] = {0.f};
	acc_data tempacc = acc;
	// ���ٶ�
	gyro.x = gyro.x * DEG2RAD;	/* ��ת���� */
	gyro.y = gyro.y * DEG2RAD;
	gyro.z = gyro.z * DEG2RAD;

	/* ���ٶȼ������Чʱ,���ü��ٶȼƲ���������*/
	if((acc.x != 0.0f) || (acc.y != 0.0f) || (acc.z != 0.0f))
	{
		/*��λ�����ټƲ���ֵ*/
		normalise = invSqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
		acc.x *= normalise;
		acc.y *= normalise;
		acc.z *= normalise;

		// ���������
		rMat[2][0] = 2.0f * (q1*q3 + -q0*q2);
	  rMat[2][1] = 2.0f * (q2*q3 - -q0*q1);
	  rMat[2][2] = 1.0f - 2.0f * q1*q1 - 2.0f * q2*q2;
	
 		
		
		/*���ټƶ�ȡ�ķ������������ټƷ���Ĳ�ֵ����������˼���*/
		ex = (acc.y * rMat[2][2] - acc.z * rMat[2][1]);
		ey = (acc.z * rMat[2][0] - acc.x * rMat[2][2]);
		ez = (acc.x * rMat[2][1] - acc.y * rMat[2][0]);
		
		/*����ۼƣ�����ֳ������*/
		exInt += Ki * ex * dt ;  
		eyInt += Ki * ey * dt ;
		ezInt += Ki * ez * dt ;
		
		/*�ò���������PI����������ƫ�����������ݶ����е�ƫ����*/
		gyro.x += Kp * ex + exInt;
		gyro.y += Kp * ey + eyInt;
		gyro.z += Kp * ez + ezInt;
	}
	/* һ����������㷨����Ԫ���˶�ѧ���̵���ɢ����ʽ�ͻ��� */
	float q0Last = q0;
	float q1Last = q1;
	float q2Last = q2;
	float q3Last = q3;
	q0 += (-q1Last * gyro.x - q2Last * gyro.y - q3Last * gyro.z) * halfT;
	q1 += ( q0Last * gyro.x + q2Last * gyro.z - q3Last * gyro.y) * halfT;
	q2 += ( q0Last * gyro.y - q1Last * gyro.z + q3Last * gyro.x) * halfT;
	q3 += ( q0Last * gyro.z + q1Last * gyro.y - q2Last * gyro.x) * halfT;
	
	/*��λ����Ԫ��*/
	normalise = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= normalise;
	q1 *= normalise;
	q2 *= normalise;
	q3 *= normalise;
	
	imuComputeRotationMatrix();	/*������ת����*/
	
	/*����roll pitch yaw ŷ����*/
	//state->attitude.x = -asinf(rMat[2][0]) * RAD2DEG; 
	//state->attitude.y = atan2f(rMat[2][1], rMat[2][2]) * RAD2DEG;
	//state->attitude.z = atan2f(rMat[1][0], rMat[0][0]) * RAD2DEG;
	// 		*yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw

 	if (!isGravityCalibrated)	/*δУ׼*/
	{		
	
//		accBuf[0] = tempacc.x* rMat[0][0] + tempacc.y * rMat[0][1] + tempacc.z * rMat[0][2];	/*accx*/
//		accBuf[1] = tempacc.x* rMat[1][0] + tempacc.y * rMat[1][1] + tempacc.z * rMat[1][2];	/*accy*/
		// ���¼��ٶ��ڵ���ο�����ϵ��ֵΪbaseAcc
		accBuf[2] = tempacc.x* rMat[2][0] + tempacc.y * rMat[2][1] + tempacc.z * rMat[2][2];	/*accz*/
		calBaseAcc(accBuf);		/*���㾲̬���ٶ�*/				
	}
}


static void calBaseAcc(float* acc)	/*���㾲̬���ٶ�*/
{
	// acc��ʾ������z���ֵ������ת����任����z���ͶӰ����
	static u16 cnt = 0;
	static float accZMin = 1.5;
	static float accZMax = 0.5;
	static float sumAcc[3] = {0.f};
	
	
	for(u8 i=0; i<3; i++)
		sumAcc[i] += acc[i];
		
	if(acc[2] < accZMin)	accZMin = acc[2];
	if(acc[2] > accZMax)	accZMax = acc[2];
	
	if(++cnt >= ACCZ_SAMPLE) /*��������*/
	{
		cnt = 0;
		maxError = accZMax - accZMin;
		accZMin = 1.5;
		accZMax = 0.5;
		if(maxError < 0.015f)
		{
			for(u8 i=0; i<3; i++)
				baseAcc[i] = sumAcc[i] / ACCZ_SAMPLE;
			isGravityCalibrated = 1;
		}
		for(u8 i=0; i<3; i++)		
			sumAcc[i] = 0.f;		
	}	
}

/*���嵽����
��δ���ʵ���˴ӻ�������ϵ����������ϵ��ת�������岽�����£�

1. ���ȣ�����������ϵ�µļ��ٶ�������acc_data * v��ת������������ϵ�¡�����ʹ������ת����rMat������������ϵ�µļ��ٶ�����ת������������ϵ�¡�����ʵ���ǽ���������ϵ�µļ��ٶ�������v->x, v->y, v->z���ֱ������ת�����ÿһ�У��õ���������ϵ�µļ��ٶ�������x, y, z����

2. ���ţ�������ת��������ƫ���ǣ�yawRad�����������cos��sinֵ��

3. Ȼ�󣬽���������ϵ�µļ��ٶ�������x, y, z��������ת���õ��ڵ�������ϵ�µļ��ٶ�������vx, vy, z��������ʹ����cos��sinֵ��������ת��

4. ��󣬽����ٶ�������y����ȡ����v->y = -vy��������ȥ�������ٶȣ�baseAcc[2] * 980.f�����õ���ʵ�ļ��ٶ�������

�ܵ���˵����δ���ʵ���˴ӻ�������ϵ����������ϵ��ת����������ת����ļ����ǹؼ���

*/
void imuTransformVectorBodyToEarth(acc_data * v)
{
    /* From body frame to earth frame */
		// ������ٶ���x��ķ��� ÿһ�����ϵķ���,���Ե����п��Ա�ʾ��z���ͶӰ

		// �õ���������ϵ�µļ��ٶ�������x, y, z)
    const float x = rMat[0][0] * v->x + rMat[0][1] * v->y + rMat[0][2] * v->z;
    const float y = rMat[1][0] * v->x + rMat[1][1] * v->y + rMat[1][2] * v->z;
    const float z = rMat[2][0] * v->x + rMat[2][1] * v->y + rMat[2][2] * v->z;
	/*
	ͨ������yawRad��ƫ���ǣ����õ�cosy��siny���ֱ����cos(yawRad)��sin(yawRad)��
	���ţ�ͨ����x��y�ֱ����cosy��siny���õ��µ�vx��vy��
	��󣬽��������������x��y��z�ֱ�ֵ��ԭʼ������x��y��z�����ҽ�z��ȥbaseAcc[2] * 980.f��ȥ���������ٶȵ�Ӱ�졣
	*/
	// ������˵��������[��Ҫ������ԭ����ǵ�����ת],x,yѡַ��ĵ�������ϵʵ���������ǵ�x,y��һ��ƫ����yawRad[ͨ�����Ҿ������],ȡx,y�����ϵķ����õ���׼ȷ��vx,vy
		float yawRad = atan2f(rMat[1][0], rMat[0][0]);
		float cosy = cosf(yawRad);
		float siny = sinf(yawRad);
		float vx = x * cosy + y * siny;
		float vy = y * cosy - x * siny;	
		
		v->x = vx;
		v->y = -vy;
		v->z = z - baseAcc[2] *  980.f;	/*ȥ���������ٶ�*/
}

/*���򵽻���*/
/**

Ϊʲô���嵽������Ҫ����yawRad��ƫ�������������򵽻���ȷû�п���

������Ϊ�ڻ��嵽�����ת���У�
��Ҫ����������ϵ�µļ��ٶ�����ת������������ϵ�£�
����������ϵ�ǹ̶��ģ���Ҫ����ƫ���ǵ�Ӱ�졣
ƫ�����ǻ�������ϵ��x�����������ϵ�µı���֮��ļнǣ�
��Ҫ�����ٶ�������ת��Ӧ�ĽǶȲ��ܵõ���ȷ�ĵ�������ϵ�µļ��ٶ�������

���ڵ��򵽻����ת���У�
��Ҫ����������ϵ�µļ��ٶ�����ת������������ϵ�£�
��ʱƫ���ǵ�Ӱ���Ѿ�����������ת��������
����˲���Ҫ�ٴο���ƫ���ǵ�������




����ԭ��
���嵽������Ҫ����yawRad��ƫ��������������Ϊ�ڻ�������ϵ�£�ƫ�����ǻ����ƴ�ֱ�ڻ��������ת�ĽǶȣ���������һ����ת�����壬�����ϵ��κ�һ���㶼�����ŵ������ת�����ϱ仯�䳯����ˣ��������һ���ط�������һ���ط�ʱ����Ҫ���ǵ������ת�Ի����Ӱ�죬�Ա�֤����ķ��з���������ʵ�ʷ���һ�¡�

�����򵽻�������Ҫ����yawRad��ƫ��������������Ϊ�ڵ�������ϵ�£�ƫ����������ڵ��򱱼���ĽǶȣ������򱱼�����һ���̶��ĵ㣬�������ŵ������ת�������仯����ˣ��������ϵ�һ������Ҫ��������ͨ�Ż��ߵ���ʱ��ֻ��Ҫ֪���õ�����ڵ��򱱼����λ�ã��Ϳ��Լ���������ƫ���ǣ�������Ҫ���ǵ������ת�Ի����Ӱ�졣
*/
void imuTransformVectorEarthToBody(acc_data * v)
{
    v->y = -v->y;

    /* From earth frame to body frame */
    const float x = rMat[0][0] * v->x + rMat[1][0] * v->y + rMat[2][0] * v->z;
    const float y = rMat[0][1] * v->x + rMat[1][1] * v->y + rMat[2][1] * v->z;
    const float z = rMat[0][2] * v->x + rMat[1][2] * v->y + rMat[2][2] * v->z;

    v->x= x;
    v->y = y;
    v->z = z;
}

