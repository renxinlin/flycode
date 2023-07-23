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
