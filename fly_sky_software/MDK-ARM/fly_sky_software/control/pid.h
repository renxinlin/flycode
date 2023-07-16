#ifndef __PID_H
#define __PID_H

#include "stm32f1xx.h"
#include "data.h"

#define DEFAULT_PID_INTEGRATION_LIMIT 		500.0 //Ĭ��pid�Ļ����޷�
#define DEFAULT_PID_OUTPUT_LIMIT      		0.0	  //Ĭ��pid����޷���0Ϊ���޷�



/*pid�ṹ���ʼ��*/
void pidInit(pid_data* pid, const float desired, const pidInit_data pidParam, const float dt);
void pidSetIntegralLimit(pid_data* pid, const float limit);/*pid�����޷�����*/
void pidSetOutputLimit(pid_data* pid, const float limit);
void pidSetDesired(pid_data* pid, const float desired);	/*pid��������ֵ*/
float pidUpdate(pid_data* pid, const float error);			/*pid����*/
float pidGetDesired(pid_data* pid);	/*pid��ȡ����ֵ*/
void pidReset(pid_data* pid);			/*pid�ṹ�帴λ*/
void pidSetError(pid_data* pid, const float error);/*pidƫ������*/
void pidSetKp(pid_data* pid, const float kp);		/*pid Kp����*/
void pidSetKi(pid_data* pid, const float ki);		/*pid Ki����*/
void pidSetKd(pid_data* pid, const float kd);		/*pid Kd����*/
void pidSetDt(pid_data* pid, const float dt);		/*pid dt����*/

#endif /* __PID_H */