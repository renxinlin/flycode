

#ifndef __INNER_CONTROL_H
#define __INNER_CONTROL_H

#include "pid.h"
#include "data.h"


extern pid_data pidAngleRoll;
extern pid_data pidAnglePitch;
extern pid_data pidAngleYaw;

extern pid_data pidRateRoll;
extern pid_data pidRatePitch;
extern pid_data pidRateYaw;


void attitudeControlInit(float rateDt, float angleDt);

void attitudeRatePID(acc_data *actualRate,attitude_data *desiredRate,control_data *output);
void attitudeAnglePID(attitude_data *actualAngle,attitude_data *desiredAngle,acc_data *outDesiredRate);
void attitudeControllerResetRollAttitudePID(void);
void attitudeControllerResetPitchAttitudePID(void);
void attitudeResetAllPID(void);
void attitudePIDwriteToConfigParam(void);
#endif