/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "adc.h"
#include "command.h"

/* USER CODE BEGIN 0 */
#define ADC_SAMPLE_NUM	10

uint16_t adc_value[5*ADC_SAMPLE_NUM];//ADC采集值存放缓冲区
 #define ABS(x) 		(((x) < 0) ? (-x) : (x))

#define MID_DB_THRUST		150	
#define MID_DB_YAW			300	
#define MID_DB_PITCH		150
#define MID_DB_ROLL			150

//摇杆上下量程死区值（ADC值）
#define DB_RANGE			10

//获取摇杆方向时定义在中间的范围值（ADC值）
#define DIR_MID_THRUST		800
#define DIR_MID_YAW			800
#define DIR_MID_PITCH		800
#define DIR_MID_ROLL		800

uint16_t getAdcValue(uint8_t axis);
int deadband(int value, const int threshold);

static uint8_t isInit;
static joystickParam_t* jsParam;
/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PA0-WKUP     ------> ADC1_IN0
    PA1     ------> ADC1_IN1
    PA2     ------> ADC1_IN2
    PA3     ------> ADC1_IN3
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PA0-WKUP     ------> ADC1_IN0
    PA1     ------> ADC1_IN1
    PA2     ------> ADC1_IN2
    PA3     ------> ADC1_IN3
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);

  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/*去除死区函数*/
int deadband(int value, const int threshold)
{
	
	if (ABS(value) < threshold)
	{
		value = 0;
	}
	else if (value > 0)
	{
		value -= threshold;
	}
	else if (value < 0)
	{
		value += threshold;
	}
	return value;
}

/*ADC值转换成飞控数据百分比*/
void ADCtoFlyDataPercent(control_data *percent)
{
	uint16_t adcValue;
	
	//THRUST 300 200 100
	adcValue = getAdcValue(ADC_THRUST) - jsParam->thrust.mid;
	adcValue = deadband(adcValue,MID_DB_THRUST);
	if(adcValue>=0)
		// 去除摇杆中间死区和上下量程死区
		percent->thrust = (float)adcValue/(jsParam->thrust.range_pos-MID_DB_THRUST-DB_RANGE);
	else
		percent->thrust = (float)adcValue/(jsParam->thrust.range_neg-MID_DB_THRUST-DB_RANGE);
	
	//ROLL
	adcValue = getAdcValue(ADC_ROLL) - jsParam->roll.mid;
	adcValue = deadband(adcValue, MID_DB_ROLL);
	if(adcValue >= 0)
		percent->roll = (float)adcValue/(jsParam->roll.range_pos-MID_DB_ROLL-DB_RANGE);
	else
		percent->roll = (float)adcValue/(jsParam->roll.range_neg-MID_DB_ROLL-DB_RANGE);
	
	//PITCH
	adcValue = getAdcValue(ADC_PITCH) - jsParam->pitch.mid;
	adcValue = deadband(adcValue, MID_DB_PITCH);
	if(adcValue >= 0)
		percent->pitch = (float)adcValue/(jsParam->pitch.range_pos-MID_DB_PITCH-DB_RANGE);
	else
		percent->pitch = (float)adcValue/(jsParam->pitch.range_neg-MID_DB_PITCH-DB_RANGE);
	
	//YAW
	adcValue = getAdcValue(ADC_YAW) - jsParam->yaw.mid;
	adcValue = deadband(adcValue, MID_DB_YAW);
	if(adcValue >= 0)
		percent->yaw = (float)adcValue/(jsParam->yaw.range_pos-MID_DB_YAW-DB_RANGE);
	else
		percent->yaw = (float)adcValue/(jsParam->yaw.range_neg-MID_DB_YAW-DB_RANGE);
}
/* USER CODE END 1 */



//ADC均值滤波
void ADC_Filter(u16* adc_val)
{
	uint16_t i=0;
	uint32_t sum[5]={0,0,0,0};
	
	for(;i<ADC_SAMPLE_NUM;i++)
	{
		sum[0]+=adc_value[5*i+0];
		sum[1]+=adc_value[5*i+1];
		sum[2]+=adc_value[5*i+2];
		sum[3]+=adc_value[5*i+3];
		sum[4]+=adc_value[5*i+4];
	}
	adc_val[0]=sum[0]/ADC_SAMPLE_NUM;
	adc_val[1]=sum[1]/ADC_SAMPLE_NUM;
	adc_val[2]=sum[2]/ADC_SAMPLE_NUM;
	adc_val[3]=sum[3]/ADC_SAMPLE_NUM;
	adc_val[4]=sum[4]/ADC_SAMPLE_NUM;
}


uint16_t getAdcValue(uint8_t axis)
{
	uint32_t sum=0;
	for(u8 i=0;i<ADC_SAMPLE_NUM;i++)
	{
		sum += adc_value[5*i+axis];
	}
	return sum/ADC_SAMPLE_NUM;
}




