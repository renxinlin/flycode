#ifndef __DELAY_H
#define __DELAY_H
#include "stm32f1xx_hal.h"
#endif

void usTickInit(void);
void delay_us(uint32_t us);
uint32_t currcent_us_timestamp(void);
