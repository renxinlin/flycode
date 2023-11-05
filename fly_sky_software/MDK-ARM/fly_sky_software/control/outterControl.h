#ifndef __OUTTER_CONTROL_H
#define __OUTTER_CONTROL_H
#include "data.h"
#include "pid.h"

extern pid_data pidVX;
extern pid_data pidVY;
extern pid_data pidVZ;

extern pid_data pidX;
extern pid_data pidY;
extern pid_data pidZ;

void positionControlInit(float ratePidDt, float posPidDt);
void positionResetAllPID(void);
void positionController(float* thrust, attitude_data *attitude, expect_data *expect_data, const self_data *self_data, float dt);
void positionPIDwriteToConfigParam(void);

#endif
