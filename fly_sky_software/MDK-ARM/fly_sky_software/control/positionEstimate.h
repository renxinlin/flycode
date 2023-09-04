#ifndef __POSITION_H
#define __POSITION_H

#include "imu.h"
#include "data.h"
#include "math.h"
#include "maths.h"


typedef struct
{
	float vAccDeadband; /* 加速度死区 */
	float accBias[3];	/* 加速度 偏置(cm/s/s)*/
	float acc[3];		/* 估测加速度 单位(cm/s/s)*/
	float vel[3];		/* 估测速度 单位(cm/s)*/
	float pos[3]; 		/* 估测位移 单位(cm)*/
} estimator_data;
/**

气压计的权重需要根据具体情况进行调整。一般来说，气压计的权重应该在0.1到0.3之间。如果飞行器在较高的高度上飞行，气压计的权重可以适当增加，如果在低高度上飞行，可以适当降低气压计的权重。

确定气压计的权重需要考虑以下因素：

1. 飞行器的高度范围：如果飞行器在较高的高度上飞行，气压计的权重可以适当增加，如果在低高度上飞行，可以适当降低气压计的权重。

2. 气压计的精度：如果气压计的精度较高，可以适当增加其权重。

3. 其他传感器的精度：如果其他传感器的精度较高，可以适当降低气压计的权重。

4. 飞行器的动态特性：如果飞行器的动态特性较大，可以适当降低气压计的权重，以减少其对飞行器的影响。

综合考虑以上因素，可以根据实际情况进行调整，以达到最佳的飞行效果。
*/

#define ACC_LIMIT			(1000.f)/*加速度限幅 单位cm/s/s*/
#define ACC_LIMIT_MAX		(1800.f)/*最大加速度限幅 单位cm/s/s*/
#define VELOCITY_LIMIT		(130.f)	/*速度限幅 单位cm/s*/
#define VELOCITY_LIMIT_MAX	(500.f)	/*最大速度限幅 单位cm/s*/

#define GRAVITY_CMSS 		(980.f)	/*重力加速度 单位cm/s/s*/
#define INAV_ACC_BIAS_ACCEPTANCE_VALUE	(GRAVITY_CMSS * 0.25f)   // Max accepted bias correction of 0.25G - unlikely we are going to be that much off anyway


static float wBaro = 0.75f;			/*气压校正权重*/
static float wAccBias = 0.001f;		/*加速度校正权重*/

void positionEstimate(sensor_data* sensorData, self_data* selfData, float dt);




static void inavFilterPredict(int axis, float dt, float acc);
/*位置校正*/
static void inavFilterCorrectPos(int axis, float dt, float e, float w);
void estRstAll(void);


#endif 
