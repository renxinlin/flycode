/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "usart.h"
#include "gpio.h"
#include "adc.h" 
#include "spi.h" 
#include "delay.h" 
#include "mpu.h" 
#include "tim.h" 
#include "fbm320.h" 
#include "imu.h"
#include "positionEstimate.h"
#include "bmp280.h" 
#include "protocol.h" 
#include "control.h" 
#include "sensors.h"
#include <stdio.h>
#include "structconfig.h"

void configParamInit(void) ;

int get_ms_count(unsigned long *count);
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define UART1_IO_OUT_Pin GPIO_PIN_9
#define UART1_IO_OUT_GPIO_Port GPIOA
#define UART1_IO_IN_Pin GPIO_PIN_10
#define UART1_IO_IN_GPIO_Port GPIOA

#define LED_Pin GPIO_PIN_0
#define LED_GPIO_Port GPIOA
#define led1_Pin GPIO_PIN_11
#define led1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
