#ifndef __IMU_H
#define __IMU_H

#include "data.h"
#include "imu.h"
#include "maths.h"
#include "stdio.h"
#include "math.h"
#define ACCZ_SAMPLE		350

// 此处是安装四元数求解标准流程定义的pid部分，不参与姿态pid部分
float Kp = 20.f;		/*需要实际调节硬件获得合适的p 经验:18 <p<30 */
float Ki = 3.f;		/*需要实际调节硬件获得合适的p  1<i<5*/

float exInt = 0.0f;
float eyInt = 0.0f;
float ezInt = 0.0f;		/*积分误差累计*/

static float q0 = 1.0f;	/*四元数*/
static float q1 = 0.0f;
static float q2 = 0.0f;
static float q3 = 0.0f;	
static float rMat[3][3];/*余弦矩阵旋转矩阵*/


static float maxError = 0.f;		/*最大误差*/
uint8_t isGravityCalibrated = 0;	/*是否校校准完成 0完成*/
static float baseAcc[3] = {0.f,0.f,1.0f};	/*静态加速度*/

void imucalcuate(void);
void calBaseAcc(float* acc);
#endif


/*计算旋转矩阵*/
void imuComputeRotationMatrix(void)
{
		// 四元数转余弦矩阵公式
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

void imuUpdate(acc_data acc, gyro_data gyro, self_data *state , float dt)	/*数据融合 互补滤波*/
{
	float normalise;
	float ex, ey, ez;
	float halfT = 0.5f * dt;
	float accBuf[3] = {0.f};
	acc_data tempacc = acc;
	// 角速度
	gyro.x = gyro.x * DEG2RAD;	/* 度转弧度 */
	gyro.y = gyro.y * DEG2RAD;
	gyro.z = gyro.z * DEG2RAD;

	/* 加速度计输出有效时,利用加速度计补偿陀螺仪*/
	if((acc.x != 0.0f) || (acc.y != 0.0f) || (acc.z != 0.0f))
	{
		/*单位化加速计测量值*/
		normalise = invSqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
		acc.x *= normalise;
		acc.y *= normalise;
		acc.z *= normalise;

		// 矩阵第三行
		rMat[2][0] = 2.0f * (q1*q3 + -q0*q2);
	  rMat[2][1] = 2.0f * (q2*q3 - -q0*q1);
	  rMat[2][2] = 1.0f - 2.0f * q1*q1 - 2.0f * q2*q2;
	
 		
		
		/*加速计读取的方向与重力加速计方向的差值，用向量叉乘计算*/
		ex = (acc.y * rMat[2][2] - acc.z * rMat[2][1]);
		ey = (acc.z * rMat[2][0] - acc.x * rMat[2][2]);
		ez = (acc.x * rMat[2][1] - acc.y * rMat[2][0]);
		
		/*误差累计，与积分常数相乘*/
		exInt += Ki * ex * dt ;  
		eyInt += Ki * ey * dt ;
		ezInt += Ki * ez * dt ;
		
		/*用叉积误差来做PI修正陀螺零偏，即抵消陀螺读数中的偏移量*/
		gyro.x += Kp * ex + exInt;
		gyro.y += Kp * ey + eyInt;
		gyro.z += Kp * ez + ezInt;
	}
	/* 一阶龙格库塔算法，四元数运动学方程的离散化形式和积分 */
	float q0Last = q0;
	float q1Last = q1;
	float q2Last = q2;
	float q3Last = q3;
	q0 += (-q1Last * gyro.x - q2Last * gyro.y - q3Last * gyro.z) * halfT;
	q1 += ( q0Last * gyro.x + q2Last * gyro.z - q3Last * gyro.y) * halfT;
	q2 += ( q0Last * gyro.y - q1Last * gyro.z + q3Last * gyro.x) * halfT;
	q3 += ( q0Last * gyro.z + q1Last * gyro.y - q2Last * gyro.x) * halfT;
	
	/*单位化四元数*/
	normalise = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= normalise;
	q1 *= normalise;
	q2 *= normalise;
	q3 *= normalise;
	
	imuComputeRotationMatrix();	/*计算旋转矩阵*/
	
	/*计算roll pitch yaw 欧拉角*/
	//state->attitude.x = -asinf(rMat[2][0]) * RAD2DEG; 
	//state->attitude.y = atan2f(rMat[2][1], rMat[2][2]) * RAD2DEG;
	//state->attitude.z = atan2f(rMat[1][0], rMat[0][0]) * RAD2DEG;
	// 		*yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw

 	if (!isGravityCalibrated)	/*未校准*/
	{		
	
//		accBuf[0] = tempacc.x* rMat[0][0] + tempacc.y * rMat[0][1] + tempacc.z * rMat[0][2];	/*accx*/
//		accBuf[1] = tempacc.x* rMat[1][0] + tempacc.y * rMat[1][1] + tempacc.z * rMat[1][2];	/*accy*/
		// 更新加速度在地球参考坐标系的值为baseAcc
		accBuf[2] = tempacc.x* rMat[2][0] + tempacc.y * rMat[2][1] + tempacc.z * rMat[2][2];	/*accz*/
		calBaseAcc(accBuf);		/*计算静态加速度*/				
	}
}


static void calBaseAcc(float* acc)	/*计算静态加速度*/
{
	// acc表示陀螺仪z轴的值经过旋转矩阵变换后在z轴的投影长度
	static u16 cnt = 0;
	static float accZMin = 1.5;
	static float accZMax = 0.5;
	static float sumAcc[3] = {0.f};
	
	
	for(u8 i=0; i<3; i++)
		sumAcc[i] += acc[i];
		
	if(acc[2] < accZMin)	accZMin = acc[2];
	if(acc[2] > accZMax)	accZMax = acc[2];
	
	if(++cnt >= ACCZ_SAMPLE) /*缓冲区满*/
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

/*机体到地球
这段代码实现了从机体坐标系到地球坐标系的转换，具体步骤如下：

1. 首先，将机体坐标系下的加速度向量（acc_data * v）转换到地球坐标系下。这里使用了旋转矩阵（rMat）将机体坐标系下的加速度向量转换到地球坐标系下。具体实现是将机体坐标系下的加速度向量（v->x, v->y, v->z）分别乘以旋转矩阵的每一行，得到地球坐标系下的加速度向量（x, y, z）。

2. 接着，根据旋转矩阵计算出偏航角（yawRad），并计算出cos和sin值。

3. 然后，将地球坐标系下的加速度向量（x, y, z）进行旋转，得到在地球坐标系下的加速度向量（vx, vy, z）。这里使用了cos和sin值来进行旋转。

4. 最后，将加速度向量在y轴上取反（v->y = -vy），并减去重力加速度（baseAcc[2] * 980.f），得到真实的加速度向量。

总的来说，这段代码实现了从机体坐标系到地球坐标系的转换，其中旋转矩阵的计算是关键。

*/
void imuTransformVectorBodyToEarth(acc_data * v)
{
    /* From body frame to earth frame */
		// 机体加速度在x轴的分量 每一行轴上的分量,所以第三行可以表示在z轴的投影

		// 得到地球坐标系下的加速度向量（x, y, z)
    const float x = rMat[0][0] * v->x + rMat[0][1] * v->y + rMat[0][2] * v->z;
    const float y = rMat[1][0] * v->x + rMat[1][1] * v->y + rMat[1][2] * v->z;
    const float z = rMat[2][0] * v->x + rMat[2][1] * v->y + rMat[2][2] * v->z;
	/*
	通过计算yawRad（偏航角）来得到cosy和siny，分别代表cos(yawRad)和sin(yawRad)。
	接着，通过将x和y分别乘以cosy和siny，得到新的vx和vy。
	最后，将修正后的向量的x、y、z分别赋值给原始向量的x、y、z，并且将z减去baseAcc[2] * 980.f，去除重力加速度的影响。
	*/
	// 总体来说就是修正[需要修正的原因就是地球自转],x,y选址后的地球坐标系实际上与真是的x,y有一个偏航角yawRad[通过余弦矩阵算出],取x,y在其上的分量得到更准确的vx,vy
		float yawRad = atan2f(rMat[1][0], rMat[0][0]);
		float cosy = cosf(yawRad);
		float siny = sinf(yawRad);
		float vx = x * cosy + y * siny;
		float vy = y * cosy - x * siny;	
		
		v->x = vx;
		v->y = -vy;
		v->z = z - baseAcc[2] *  980.f;	/*去除重力加速度*/
}

/*地球到机体*/
/**

为什么机体到地球需要考虑yawRad的偏航角修正，地球到机体确没有考虑

这是因为在机体到地球的转换中，
需要将机体坐标系下的加速度向量转换到地球坐标系下，
而地球坐标系是固定的，需要考虑偏航角的影响。
偏航角是机体坐标系下x轴与地球坐标系下的北向之间的夹角，
需要将加速度向量旋转相应的角度才能得到正确的地球坐标系下的加速度向量。

而在地球到机体的转换中，
需要将地球坐标系下的加速度向量转换到机体坐标系下，
此时偏航角的影响已经被考虑在旋转矩阵中了
，因此不需要再次考虑偏航角的修正。




修正原因
机体到地球需要考虑yawRad的偏航角修正，是因为在机体坐标系下，偏航角是机体绕垂直于机体的轴旋转的角度，而地球是一个旋转的天体，地球上的任何一个点都会随着地球的自转而不断变化其朝向。因此，当机体从一个地方飞往另一个地方时，需要考虑地球的自转对机体的影响，以保证机体的飞行方向与地面的实际方向一致。

而地球到机体则不需要考虑yawRad的偏航角修正，是因为在地球坐标系下，偏航角是相对于地球北极点的角度，而地球北极点是一个固定的点，不会随着地球的自转而发生变化。因此，当地面上的一个点需要与机体进行通信或者导航时，只需要知道该点相对于地球北极点的位置，就可以计算出机体的偏航角，而不需要考虑地球的自转对机体的影响。
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

