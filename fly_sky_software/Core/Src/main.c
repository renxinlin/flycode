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
#define VREF            4.2f

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
	while(mpu_dmp_init())  //���ٶȴ������Լ�
	{
			HAL_Delay(200);
			printf("canot init dmp\r\n");
	} 
	// FBM320_Init();
	BMP280_Init();
 	HAL_Delay(200);
	HAL_GPIO_TogglePin(led1_GPIO_Port, led1_Pin);
  NRF24l01_Init( );		// ���nRF24L01	
 	HAL_Delay(200);
	HAL_GPIO_TogglePin(led1_GPIO_Port, led1_Pin);

	SENSER_FLAG_SET(BAR_OFFSET);//У׼��ѹ�ƿ���
	uint32_t pwmValue = 0;
	// ��ʼ������ configparam ��Ҫ���ڳ�ʼ��pid
	HAL_Delay(200);
	HAL_GPIO_TogglePin(led1_GPIO_Port, led1_Pin);
	sensorsInit();
	// pid��ʼ��
	HAL_Delay(200);
	HAL_GPIO_TogglePin(led1_GPIO_Port, led1_Pin);
	configParamInit();
 	stateControlInit();
	HAL_GPIO_TogglePin(led1_GPIO_Port, led1_Pin);
	
	// ��ȫ����
	remeote_recieve = 0;
	systime_ms=0;		
  systime = 0;
	last_remeote_recieve_time = -1000;
  while (1)
  {
		// ��ȫ���ƣ� 2s���յ�ң���������򱣳�״̬,����ͣ��
		if((systime - last_remeote_recieve_time )>2){
			remeote_recieve = 0;
		}else{
			remeote_recieve=1;
		}
			
			
		if(counter_2ms>=2){

			 counter_2ms = 0;
			// 2ms����ִ��
				if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0){      
						self.attitude.x = roll;
						self.attitude.y = pitch;
						self.attitude.z = yaw;
					//	printf("����Ƕȣ�roll = %f pitch = %f yaw = %f\r\n",roll,pitch,yaw);
				}
			stateControl(&control_info, &sensors, &self, &expect_set_point, tick);
						//	printf(" roll = %d pitch = %d  yaw = %d \r\n",control_info.roll,control_info.pitch,control_info.yaw);

			tick++;
			if(tick>1){
				tick = 0;
			}
		
			// �ɻ����� 1Ϊ���� 0Ĭ������
			control_info.pitch = remeote_recieve==1 && withBias && remoter.rcLock == 1 ? control_info.pitch : 0;
			control_info.roll = remeote_recieve==1 && withBias && remoter.rcLock == 1 ? control_info.roll : 0;
			control_info.yaw = remeote_recieve==1 && withBias && remoter.rcLock == 1 ? control_info.yaw : 0;
			// 0��65535
			control_info.thrust = remeote_recieve && withBias && remoter.rcLock == 1 ? control_info.thrust : 0;
			powerControl(&control_info);

		}

		if(counter_5ms>=5){

				counter_5ms = 0;
				// ��ѹ�����ݻ�ȡ
				// 5ms����ִ��
			BMP280_CalTemperatureAndPressureAndAltitude(&temperature, &pressure, &asl);
			sensors.baro.temperature = (float) temperature;
			sensors.baro.pressure = (float) pressure;
			sensors.baro.asl = (float) asl;
			// 2200
			// printf("height is %f \r\n",sensors.baro.asl);

		}
		if(counter_4ms>=4){

			counter_4ms = 0;
			withBias = sensorsDataGet();			
			if(withBias){
					//imuUpdate(sensors.acc,sensors.gyro,&self,0.004f);
				// �жϻ������ٶ��Ƿ�������
				// [04:06:10.001]�ա���accscale is 1.092690 
					//printf("��Լ��ٶ� %f  \r\n",sensors.acc.z);��Լ��ٶ� 0.992496  
			}	
			// baseacc is 0.999919
			// ��������߶�
		  positionEstimate(&sensors, &self, 0.004f);
		}

		if(counter_10ms>=10){
			counter_10ms = 0;

			commanderGetSetpoint(&expect_set_point, &self);

			// ң�������ݴ��� 

		}

		if(counter_20ms>=20){
			counter_20ms = 0;
			// 20ms����ִ��
		}
		if(counter_50ms>=50){
			if(withBias){
				HAL_GPIO_TogglePin(led1_GPIO_Port, led1_Pin);
			}
				counter_50ms = 0;
				Remote_Data_Send();
				// ��ȡ��ѹ
				if (HAL_ADC_Start(&hadc1) != HAL_OK)
        {
            Error_Handler();
        }

        // �ȴ�ת�����
        if (HAL_ADC_PollForConversion(&hadc1, 1000) != HAL_OK)
        {
            Error_Handler();
        }

        // ��ȡת�����
        int32_t adc_value = HAL_ADC_GetValue(&hadc1);
        voltage = ((float)adc_value / 4096.0) * VREF;
				
			//	printf("adc is %d and voltage is %f \r\n",adc_value ,voltage);
			//if(withBias){
			//		HAL_GPIO_TogglePin(led1_GPIO_Port, led1_Pin);
			//}

 		}

	}
  /* USER CODE END 3 */
}

	void	configParamInit(void){
		// pid��ʼ��
		configParam.pidAngle.roll.kp=4; // 2.8 0.01 0.01
		configParam.pidAngle.roll.ki=0.02; // 0
		configParam.pidAngle.roll.kd=0.02;
		
		configParam.pidAngle.pitch.kp=4; //  2������ 1 ��Ƶ�� �����ڻ�p��Сһ�� �⻷p����΢��һ��
		configParam.pidAngle.pitch.ki=0.02;
		configParam.pidAngle.pitch.kd=0.02;
		

/////////////////////
		configParam.pidRate.roll.kp=65.0; //    10��ת��Ʈ   40�ָ�����ǿ��60~70����ֵ��80��Ƶ��  100~200�������ػζ�  ��ɢ 
		configParam.pidRate.roll.ki=0.1;
		configParam.pidRate.roll.kd=0.781; // 6
		// ���бȽ���,����Ƕȵ���,Ӧ�û��кܸ�Ƶ��΢С������Ҫ�޸�
			// 125 8 5
		// ���бȽ���,����Ƕȵ���,�ܸ�Ƶ��΢С�������

			// p 3~5 60~80
			// 160 8 5
			// 200 8 10
		// ��������һ���õ�ֵʱС���ȵĵȷ��ζ�
		configParam.pidRate.pitch.kp=65.0; //[120 ,135]
		// 200~240�ζ� 400�ζ�����
		// 100 150  300 400 600 800 1000
		// 170 4.1V�ζ�
		// 150���ζ�  ����������100~200֮��
		// 130 �������䵽һ��λ�ú�,С��Χ�ζ�
		// 120����ô���� С��Χ�ζ� ��ѹ3.9V
		// ֻ���ڻ� P ��ʱ������Ỻ������һ�������µ����������������������ϵͳ���ٶȾ��
		// i��Ŀ
		// ���бȽ���,����Ƕȵ���,Ӧ�û��кܸ�Ƶ��΢С������Ҫ�޸�
		//==========================8.30======================================
		
		// 100 ~800 ���ǻζ� ����һ���������
		// P=0.8ʱ���Թ۲쵽�Ƚ����Եĵȷ����ˣ�P=0.2ʱ�������Եõ�����������PӦ����0.2~0.8֮�䡣�ٽ��𵴵����P��Ϊ�𵴵����տ�ʼ�𵴵ĵ�
		// ����������Ķ�������0��λ�á�������ֻ��ѡ����ѡ��P,D������I������������
		// ��D�����ԣ���ȻD����Ҳ������𵴣����Ǵ�ʱ����Dʱ��P����ʱ���𵴾ͺ�С�������ԾͿ��Կ�������D�������𵴼�С������Ĺ���
		//=======�𽥼Ӵ�Pֱ����ʼ�����ȷ��𵴣�Ȼ��P���䣬����D������
		// �ο�https://www.amobbs.com/thread-5554367-1-1.html
		
		// 130ʱ�ȶ������ڵĹؼ�ʱ�ҳ�p������,��d������� �Լ����ٻ����ǿ����Ż����⻷
		// �Ѿ�֤�����ٶȻ���Ҫ������Ӧ�仯,������Ҫ�ӻ����������ֵ���
		configParam.pidRate.pitch.ki=0.1;
		configParam.pidRate.pitch.kd=0.78; // 6.5
		////////////////////////////////////////////////////////////
		configParam.pidAngle.yaw.kp=2;//  20
		configParam.pidAngle.yaw.ki=0.5;
		configParam.pidAngle.yaw.kd=0;
		
		configParam.pidRate.yaw.kp=30; // 200 80
		configParam.pidRate.yaw.ki=0.2; // 18.5
		configParam.pidRate.yaw.kd=0.0; // 0.0
		////////////////////////////////////////////////////////////

		configParam.pidPos.z.kp=6.0;
		configParam.pidPos.z.ki=0.0;
		configParam.pidPos.z.kd=4.5;
				
		configParam.pidPos.vz.kp=100.0;
		configParam.pidPos.vz.ki=150.0;
		configParam.pidPos.vz.kd=10.0;
		
				
		// ��������
		configParam.thrustBase=(65535-1)/2 ;
		// ƫ��
		configParam.trimR=0.f;
		configParam.trimP=0.f; // �ɻ���y�������
		/////////////////////////////////////////////  110 2 2
		
		configParam.pidPos.x.kp=4.0;
		configParam.pidPos.x.ki=0.0;
		configParam.pidPos.x.kd=0.6;
		configParam.pidPos.y.kp=4.0;
		configParam.pidPos.y.ki=0.0;
		configParam.pidPos.y.kd=0.6;
				

		configParam.pidPos.vx.kp=4.5;
		configParam.pidPos.vx.ki=0.0;
		configParam.pidPos.vx.kd=0.0;
		
		configParam.pidPos.vy.kp=4.5;
		configParam.pidPos.vy.ki=0.0;
		configParam.pidPos.vy.kd=0.0;
		

		
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
