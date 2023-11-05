/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"
#include "data.h"
#include "delay.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */

  /*Configure GPIO pins : PBPin PBPin PBPin PBPin */
  GPIO_InitStruct.Pin = KEY1_Pin|KEY2_Pin|KEY3_Pin|KEY4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  //GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
 HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
 HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CSN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CE_GPIO_Port, &GPIO_InitStruct);

  /* pb1 nrf�ⲿ�ж� */
  GPIO_InitStruct.Pin = IRQ_Pin;
  // GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IRQ_GPIO_Port, &GPIO_InitStruct);
	
	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
	
	
	
	// ����
	GPIO_InitTypeDef key_initStruct;
  
  key_initStruct.Pin = BUTTON1_PIN | BUTTON2_PIN | BUTTON3_PIN | BUTTON4_PIN;
  key_initStruct.Mode = GPIO_MODE_IT_FALLING;
  key_initStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_GPIO_PORT, &GPIO_InitStruct);
  
  // �ж����ȼ����� 0~15 ���ȼ�ԽСԽ���� 0�������ɻ���ص�
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* USER CODE BEGIN 2 */
 

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	static int i = 0; // ΢��ģʽ
	if (GPIO_Pin == BUTTON1_PIN)
  {
    // ���ݰ��������͸��ɻ������� �Լ���ʾ����չʾ
		printf("BUTTON1_PIN ���� �ɻ�����");
		delay_us(1000);  // ������ʱ��������Ҫ������ʱʱ��
        if (HAL_GPIO_ReadPin(BUTTON_GPIO_PORT, GPIO_Pin) == GPIO_PIN_RESET) {
            if(remoter_buffer.rcLock == 0){
							// ����
							printf("����");
							remoter_buffer.rcLock=1;
						}else{
							// ����
							printf("����");

							remoter_buffer.rcLock=0;

						}
        }
  }
		if (GPIO_Pin == BUTTON2_PIN)
  {
		
					delay_us(1000);  // ������ʱ��������Ҫ������ʱʱ�� hal_delay���¿���
					if (HAL_GPIO_ReadPin(BUTTON_GPIO_PORT, GPIO_Pin) == GPIO_PIN_RESET) {
					 		if(trim == 0){
								// ΢��ģʽ����
								trim=1;
							}else{
								// ΢��ģʽ�ر�
								trim=0;

							}
					}
					
    // ����
  }
	if (GPIO_Pin == BUTTON3_PIN||GPIO_Pin == BUTTON4_PIN)
  {
		delay_us(1000);  // ������ʱ��������Ҫ������ʱʱ��
		if (HAL_GPIO_ReadPin(BUTTON_GPIO_PORT, GPIO_Pin) == GPIO_PIN_RESET) {
			if(remoter_buffer.ctrlMode == 1){	
										printf("�ڻ�����\r\n");

					remoter_buffer.ctrlMode=3;
			}else{
									remoter_buffer.ctrlMode=1;
										printf("�⻷����\r\n");


			}
		}
  }
}

/* USER CODE END 2 */
