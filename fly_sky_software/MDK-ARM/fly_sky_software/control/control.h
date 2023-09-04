#ifndef __CONZTROL_H
#define __CONZTROL_H


#include "main.h"
#include "tim.h"
#include "innerControl.h"
#include "outterControl.h"
void stateControlInit(void);
void powerControl(control_data *control);
void stateControl(control_data *control, sensor_data *sensors, self_data *self, expect_data *setpoint, const uint32_t tick);
#endif