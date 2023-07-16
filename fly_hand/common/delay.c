#include "delay.h"
#include <stdio.h>


uint32_t usTick;

void usTickInit(void)
{
	
	// 72000000 /1000000
	uint32_t freq_72M = HAL_RCC_GetHCLKFreq();
	usTick = freq_72M/10000/1000;

}
void delay_us(uint32_t us)
{
	// 1ms的计数器 72000 递减计数器
	uint32_t startUs = currcent_us_timestamp();
	while((currcent_us_timestamp() - startUs) > us);
}


uint32_t currcent_us_timestamp(void){
	uint32_t startUsStart = HAL_GetTick() *1000 + (usTick * 1000 - SysTick->VAL) / usTick;
	return startUsStart;
}
