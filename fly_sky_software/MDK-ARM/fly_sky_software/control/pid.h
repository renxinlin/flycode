#ifndef __PID_H
#define __PID_H

#include "stm32f1xx.h"
#include "data.h"

#define DEFAULT_PID_INTEGRATION_LIMIT 		500.0 //默认pid的积分限幅
#define DEFAULT_PID_OUTPUT_LIMIT      		0.0	  //默认pid输出限幅，0为不限幅



/*pid结构体初始化*/
void pidInit(pid_data* pid, const float desired, const pidInit_data pidParam, const float dt);
void pidSetIntegralLimit(pid_data* pid, const float limit);/*pid积分限幅设置*/
void pidSetOutputLimit(pid_data* pid, const float limit);
void pidSetDesired(pid_data* pid, const float desired);	/*pid设置期望值*/
float pidUpdate(pid_data* pid, const float error);			/*pid更新*/
float pidGetDesired(pid_data* pid);	/*pid获取期望值*/
void pidReset(pid_data* pid);			/*pid结构体复位*/
void pidSetError(pid_data* pid, const float error);/*pid偏差设置*/
void pidSetKp(pid_data* pid, const float kp);		/*pid Kp设置*/
void pidSetKi(pid_data* pid, const float ki);		/*pid Ki设置*/
void pidSetKd(pid_data* pid, const float kd);		/*pid Kd设置*/
void pidSetDt(pid_data* pid, const float dt);		/*pid dt设置*/

#endif /* __PID_H */