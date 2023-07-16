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
uint8_t    SENSER_OFFSET_FLAG; 

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

extern FBMTYPE FBM;
short mpudata[6] = {0};
int get_ms_count(unsigned long *count)
{
  *count = HAL_GetTick();
  return 0;
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  /* USER CODE END 1 */
		float pressure, temperature, asl;
		double pressure1, temperature1, asl1;

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();	
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
	HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin,GPIO_PIN_SET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin,GPIO_PIN_RESET);
	HAL_Delay(500);

  MX_TIM1_Init();
	MX_TIM2_Init();
	// __HAL_TIM_SET_COMPARE修改电机占空比

	usTickInit();
	MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while(MPU_Init()){
			printf("start mpu error");
	}
	// FBM320_Init();
	printf("send data start");
  bmp280Init();
	 //NRF24l01_Init( );		// 检测nRF24L01	
	
	SENSER_FLAG_SET(BAR_OFFSET);//校准气压计开启

  while (1)
  {
		HAL_Delay(1000);
		
		//BMP280_calc_values(&pressure1,  &temperature1,  &asl1);
		BMP280_ReadTemperatureAndPressure(&temperature,&pressure);
					asl=bmp280PressureToAltitude(&pressure);	/*转换成海拔*/

		printf(" next IS pressure is %f temperature IS %f asl %f\r\n",pressure,temperature,asl);
			bmp280GetData(&pressure,&temperature,&asl);
	   printf(" pressure  is %f temp is %f and asl is %f\n", pressure,temperature,asl);

		if(counter_2ms>=2){
			counter_2ms = 0;
			// 2ms任务执行
				
				sensorsDataGet();
				// 转数据结构和操作上下文
			
				// printf("mpudata %f,%hd,%hd,%hd,%hd,%hd \r\n",sensors.acc.x,mpudata[1],mpudata[2],mpudata[4],mpudata[4],mpudata[5]);
				printf("mpudata %f,%f,%f,%f,%f,%f \r\n",sensors.acc.x,sensors.acc.y,
			sensors.acc.z,sensors.gyro.x,sensors.gyro.y,sensors.gyro.z);
			imuUpdate(sensors.acc,sensors.gyro,&self,counter_5ms);
			
			//

		}
		if(counter_5ms>=5){
				counter_5ms = 0;
				// 气压计数据获取
				// 5ms任务执行
			bmp280GetData(&pressure, &temperature,  &asl);
			// todo 气压计
		}
		if(counter_10ms>=10){
			counter_10ms = 0;
			// 10ms任务执行
		}
		if(counter_20ms>=20){
			counter_20ms = 0;
			// 20ms任务执行
		}
		if(counter_50ms>=50){
			counter_50ms = 0;
			// 50ms任务执行
		 bmp280GetData(&pressure,&temperature,&asl);
	   printf(" pressure  is %f temp is %f and asl is %f\n", pressure,temperature,asl);

		 HAL_GPIO_TogglePin(led1_GPIO_Port, led1_Pin);
		//	powerControl();
		}
		
		// printf("初始化压强 %f  \r\n",FBM.InitPress);
		// printf("相对高度 %f  \r\n",FBM.Altitude);
    /* 
			while(mpu_dmp_init())  //加速度传感器自检
			{
				  HAL_Delay(200);
					printf("canot init dmp\r\n");
			}  
		*/
			// 参考现有源码
			// 调查缓冲区实现
			// 实现缓冲区
			// 实现数据结构转换对象 
			// 阅读stm32 完毕
			// 需要构建一个定时器实现cpu任务切换，不在引入os,减小cm3内核的压力
			//
			// 准备pid
			// 准备pmw
		/* 
			  while(1){
					if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0){      
			//		temp=MPU_Get_Temperature();								//得到温度值
			//		MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
				//	MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
				 		printf("三轴角度：%f-%f-%f\r\n",pitch,roll,yaw);
			//		printf("三轴加速度：%d-%d-%d\r\n",aacx,aacy,aacz);
			//		printf("三轴角角度：%d-%d-%d\r\n",gyrox,gyroy,gyroz);
						HAL_GPIO_TogglePin(led1_GPIO_Port, led1_Pin);
						HAL_Delay(500);
					}
					 */	
				//}
					//printf("z\r\n");
			//}
		}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  
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
