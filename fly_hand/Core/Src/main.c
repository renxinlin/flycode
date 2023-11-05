/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "delay.h"
#include "oled.h"
#include "stdio.h"
#include "nrf24l01.h"
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void toFlyData(int32_t *thrust_value,int32_t *roll_value,int32_t *pitch_value,int32_t *yaw_value);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
			 uint32_t adc_values[4]; // 保存四个通道的ADC值

 
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  usTickInit();
	NRF24l01_Init(); 
	MX_USART1_UART_Init();

	remoter_buffer.ctrlMode=2; // 油门
	int i = 0;

  while (1)
  {
		ADC_ChannelConfTypeDef sConfig;
    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5; // ADC_SAMPLETIME_71CYCLES_5
		
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
		HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    adc_values[0] =(int32_t) HAL_ADC_GetValue(&hadc1); // 
		HAL_ADC_Stop(&hadc1);

		sConfig.Channel = ADC_CHANNEL_1;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		adc_values[1] =(int32_t) HAL_ADC_GetValue(&hadc1); // 
	  HAL_ADC_Stop(&hadc1);

		sConfig.Channel = ADC_CHANNEL_2;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    adc_values[2] =(int32_t) HAL_ADC_GetValue(&hadc1); //  
 	  HAL_ADC_Stop(&hadc1);

		sConfig.Channel = ADC_CHANNEL_3;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		adc_values[3] = (int32_t)HAL_ADC_GetValue(&hadc1); //  
	  HAL_ADC_Stop(&hadc1);

	  int32_t	roll_value = adc_values[0] ;
	  int32_t	pitch_value  = adc_values[1] ;
	  int32_t	yaw_value = adc_values[2] ;
	  int32_t	thrust_value = adc_values[3] ;
		toFlyData(&thrust_value,&roll_value,&pitch_value,&yaw_value);	 
		if(trim){
			remoter_buffer.trimRoll = roll_value;
			remoter_buffer.trimPitch = pitch_value;
			remoter_buffer.yaw += 0;
			remoter_buffer.thrust = thrust_value;
		}else{
			remoter_buffer.pitch = pitch_value;
			remoter_buffer.roll = roll_value;
			remoter_buffer.yaw += yaw_value;
			remoter_buffer.thrust = thrust_value;
		}
		
		HAL_Delay(20);
		// nrf2401发送数据

		Remote_Data_Send();
		i++;
		if(i>20){
			i=0;
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		}
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
 // RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
//  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
 // PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
 // if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  //{
 //   Error_Handler();
//  }
}
void toFlyData(int32_t *thrust_value,int32_t *roll_value,int32_t *pitch_value,int32_t *yaw_value){
		// 0 == 45*45
		int up = 2025 + 3*45;
		int down = 2025-3*45;
		if(*thrust_value  <= up && *thrust_value  >= down ){
			*thrust_value = 2025;
		}
		if(*roll_value  <= up && *roll_value  >= down ){
			*roll_value = 2025;
		}
		if(*pitch_value  <= up && *pitch_value  >= down ){
			*pitch_value = 2025;
		}
		if(*yaw_value  <= up && *yaw_value  >= down ){
			*yaw_value = 2025;
		}
		// 0 ~ 4050
		*roll_value = ((2025-*roll_value)/45)/2;
		*pitch_value = ((2025-*pitch_value)/45)/2;
		*yaw_value = ((2025-*yaw_value)/45);
		// 0~4050映射到0~65535   thrust_value/4050*65535 中间油门65535/2作为基础油门 定高时飞机会将0~65535 转到1~100cm之间 默认定高50cm
		*thrust_value = *thrust_value*16.1815f;
		*thrust_value = (*thrust_value > 55000) ? 55000: *thrust_value;
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
