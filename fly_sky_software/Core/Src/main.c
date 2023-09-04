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
		float pitch,roll,yaw;
		int32_t pressure, temperature, asl;
		int32_t i=0,i1=0;
		double pressure1, temperature1, asl1;
		uint8_t withBias = 0;
	  int tick = 0;
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
  MX_TIM1_Init();
	MX_TIM2_Init();
	usTickInit();
	MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_GPIO_TogglePin(led1_GPIO_Port, led1_Pin);
				HAL_Delay(500);
	HAL_GPIO_TogglePin(led1_GPIO_Port, led1_Pin);
					HAL_Delay(500);

	while(MPU_Init()){
			HAL_GPIO_TogglePin(led1_GPIO_Port, led1_Pin);
		  printf("canot init mpu\r\n");
	}
	while(mpu_dmp_init())  //加速度传感器自检
	{
			HAL_Delay(200);
			printf("canot init dmp\r\n");
	} 
	// FBM320_Init();
	BMP280_Init();
 	HAL_Delay(200);
	HAL_GPIO_TogglePin(led1_GPIO_Port, led1_Pin);
  NRF24l01_Init( );		// 检测nRF24L01	
 	HAL_Delay(200);
	HAL_GPIO_TogglePin(led1_GPIO_Port, led1_Pin);

	SENSER_FLAG_SET(BAR_OFFSET);//校准气压计开启
	uint32_t pwmValue = 0;
	// 初始化参数 configparam 主要用于初始化pid
	HAL_Delay(200);
	HAL_GPIO_TogglePin(led1_GPIO_Port, led1_Pin);
	sensorsInit();
	// pid初始化
	HAL_Delay(200);
	HAL_GPIO_TogglePin(led1_GPIO_Port, led1_Pin);
	configParamInit();
 	stateControlInit();
  while (1)
  {
		if(counter_2ms>=2){
			 counter_2ms = 0;
			// 2ms任务执行
				if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0){      
						self.attitude.x = roll;
						self.attitude.y = pitch;
						self.attitude.z = yaw;
				 	//	printf("三轴角度：roll = %f pitch = %f yaw = %f\r\n",roll,pitch,yaw);
				}
			stateControl(&control_info, &sensors, &self, &expect_set_point, tick);
			tick++;
			if(tick>1){
				tick = 0;
			}
			if(withBias){
				powerControl(&control_info);	
			}
		}
		if(counter_5ms>=5){
				counter_5ms = 0;
				// 气压计数据获取
				// 5ms任务执行
			BMP280_CalTemperatureAndPressureAndAltitude(&temperature, &pressure, &asl);
			sensors.baro.temperature = (float) temperature;
			sensors.baro.pressure = (float) pressure;
			sensors.baro.asl = (float) asl;

		}
		if(counter_4ms>=4){
			counter_4ms = 0;
			withBias = sensorsDataGet();			
			if(withBias){
					//imuUpdate(sensors.acc,sensors.gyro,&self,0.004f);
				// 判断基础加速度是否计算完成
				// [04:06:10.001]收←◆accscale is 1.092690 
					//printf("相对加速度 %f  \r\n",sensors.acc.z);相对加速度 0.992496  
			}	
			// baseacc is 0.999919
		  positionEstimate(&sensors, &self, 0.004f);

		}
		if(counter_10ms>=10){
			counter_10ms = 0;
			commanderGetSetpoint(&expect_set_point, &self);
		}
		if(counter_20ms>=20){
			counter_20ms = 0;
			// 20ms任务执行
		}
		if(counter_50ms>=50){
				counter_50ms = 0;
			if(withBias){
					HAL_GPIO_TogglePin(led1_GPIO_Port, led1_Pin);
			}
			// 每5s  自动调试  pid+50
		}
		
		// printf("初始化压强 %f  \r\n",FBM.InitPress);
		// printf("相对高度 %f  \r\n",FBM.Altitude);
    /* 
		 
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

	void	configParamInit(void){
		// pid初始化
		configParam.pidAngle.roll.kp=5.0; // 8
		configParam.pidAngle.roll.ki=1.0;
		configParam.pidAngle.roll.kd=0.0;
		
		configParam.pidAngle.pitch.kp=5.0; // 8
		configParam.pidAngle.pitch.ki=1.0;
		configParam.pidAngle.pitch.kd=0.0;
		
		configParam.pidAngle.yaw.kp=0.0;//  20
		configParam.pidAngle.yaw.ki=0.0;
		configParam.pidAngle.yaw.kd=1.5;
/////////////////////
		configParam.pidRate.roll.kp=110.0; // 
		configParam.pidRate.roll.ki=2.0;
		configParam.pidRate.roll.kd=2.0; // 6
		// 回中比较慢,还会角度掉落,应该还有很高频很微小的震动需要修复
			// 125 8 5
		// 回中比较慢,还会角度掉落,很高频很微小的震动相比

			// 145 10 4
			// 160 8 5
			// 200 8 10
		// 我们期望一个好的值时小幅度的等幅晃动
		configParam.pidRate.pitch.kp=110.0; //[120 ,135]
		// 200~240晃动 400晃动增大
		// 100 150  300 400 600 800 1000
		// 170 4.1V晃动
		// 150不晃动  初步估计在100~200之间
		// 130 基本掉落到一个位置后,小范围晃动
		// 120不怎么掉落 小范围晃动 电压3.9V
		// 只有内环 P 的时候，四轴会缓慢的往一个方向下掉，这属于正常现象。这就是系统角速度静差。
		// i项目
		// 回中比较慢,还会角度掉落,应该还有很高频很微小的震动需要修复
		//==========================8.30======================================
		
		// 100 ~800 都是晃动 且往一个方向掉落
		// P=0.8时可以观察到比较明显的等幅震荡了，P=0.2时四轴又显得的无力，所以P应该在0.2~0.8之间。临界震荡点就是P从为震荡到，刚开始震荡的点
		// 四轴的震荡中心都不是在0度位置。所以我只能选择先选好P,D最后加入I来解决这个问题
		// 调D就是试，当然D大了也会产生震荡，但是此时不加D时光P作用时的震荡就很小，很明显就可以看出随着D的增大，震荡减小又增大的过程
		//=======逐渐加大P直到开始发生等幅震荡，然后P不变，加入D抑制震荡
		// 参考https://www.amobbs.com/thread-5554367-1-1.html
		
		// 130时稳定，现在的关键时找出p的上限,与d如何抑制 以及快速回中是靠油门还是外环
		// 已经证明角速度环需要快速响应变化,所以需要从基础油门入手调整
		configParam.pidRate.pitch.ki=2.0;
		configParam.pidRate.pitch.kd=2.0; // 6.5
		
		configParam.pidRate.yaw.kp=1; // 200 80
		configParam.pidRate.yaw.ki=0; // 18.5
		configParam.pidRate.yaw.kd=0.0; // 0.0
		/////////////////////////////////////////////  110 2 2
		
		configParam.pidPos.x.kp=4.0;
		configParam.pidPos.x.ki=0.0;
		configParam.pidPos.x.kd=0.6;
	
		configParam.pidPos.y.kp=4.0;
		configParam.pidPos.y.ki=0.0;
		configParam.pidPos.y.kd=0.6;
				
		configParam.pidPos.z.kp=6.0;
		configParam.pidPos.z.ki=0.0;
		configParam.pidPos.z.kd=4.5;
				
		configParam.pidPos.vx.kp=4.5;
		configParam.pidPos.vx.ki=0.0;
		configParam.pidPos.vx.kd=0.0;
		
		configParam.pidPos.vy.kp=4.5;
		configParam.pidPos.vy.ki=0.0;
		configParam.pidPos.vy.kd=0.0;
		
		configParam.pidPos.vz.kp=100.0;
		configParam.pidPos.vz.ki=150.0;
		configParam.pidPos.vz.kd=10.0;
		
		// 基础油门
		configParam.thrustBase=(65535-1)/2 ;
		// 偏置
		configParam.trimR=0.f;
		configParam.trimP=0.f;
		
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
