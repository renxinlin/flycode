/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#define CSN_Pin GPIO_PIN_4
#define CSN_GPIO_Port GPIOA
#define CE_Pin GPIO_PIN_0
#define CE_GPIO_Port GPIOB
#define IRQ_Pin GPIO_PIN_1
#define IRQ_GPIO_Port GPIOB
/* USER CODE END Includes */
#define BUTTON1_PIN GPIO_PIN_12
#define BUTTON2_PIN GPIO_PIN_13
#define BUTTON3_PIN GPIO_PIN_14
#define BUTTON4_PIN GPIO_PIN_15
#define BUTTON_GPIO_PORT GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */

 
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

