/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"
#include "data.h"
#include "stdio.h"
#include <stdint.h>

/* USER CODE BEGIN 0 */
#define MAX_USART_BUFFER 1
#define LENGTH 200
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);//注意把&huart1改为自己的stm32使用的串口号
  while(HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX);//检测UART发送结束				HAL_Delay(500);
	return ch;
}

uint8_t irq_buffer[MAX_USART_BUFFER];
uint8_t buffer[LENGTH];
uint8_t bufferIndex = 0;		//接收缓冲计数
/* USER CODE END 0 */

UART_HandleTypeDef huart1;
void parseData(const uint8_t* buffer, uint8_t length, uint8_t* a, uint8_t* b, uint8_t* c);
/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
	HAL_UART_Receive_IT(&huart1, (uint8_t *)&irq_buffer, 1);

  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART1_MspInit 1 */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
		HAL_UART_Receive_IT(&huart1, (uint8_t *)&irq_buffer, 1);
  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

  /* USER CODE BEGIN USART1_MspDeInit 1 */
    HAL_NVIC_DisableIRQ(USART1_IRQn);

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if(bufferIndex >= 255){
		//溢出判断
		bufferIndex = 0;
		HAL_UART_Transmit(&huart1, (uint8_t *)"数据溢出", 10,0xFFFF); 	
	}
	else{
		buffer[bufferIndex++] = irq_buffer[0];   //接收数据转存
		if((buffer[bufferIndex-1] == 0x0A)&&(buffer[bufferIndex-2] == 0x0D)) //判断结束位
		{
			// p i d 
			parseData(buffer, bufferIndex+1, &(remoter_buffer.type), &(remoter_buffer.length), &(remoter_buffer.checksum));

			// HAL_UART_Transmit(&huart1, (uint8_t *)&buffer, bufferIndex,0xFFFF); //将收到的信息发送出去
      // while(HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX);//检测UART发送结束
			//printf("remoter.type is %d ",remoter.type);
			//printf("remoter.type is %d ",remoter.length);
			//printf("remoter.type is %d ",remoter.checksum);
			bufferIndex = 0;
		}
	}
	// 重新使能中断
		HAL_UART_Receive_IT(&huart1, (uint8_t *)&irq_buffer, 1);
}


void parseData(const uint8_t* buffer, uint8_t length, uint8_t* a, uint8_t* b, uint8_t* c) {
    uint8_t i = 0;
    // 跳过空格和换行符
    while (i < length && (buffer[i] == ' ' || buffer[i] == '\r' || buffer[i] == '\n')) {
        i++;
    }
    // 解析第一个数字
    *a = 0;
    while (i < length && buffer[i] >= '0' && buffer[i] <= '9') {
        *a = (*a) * 10 + buffer[i] - '0';
        i++;
    }
    // 跳过空格和换行符
    while (i < length && (buffer[i] == ' ' || buffer[i] == '\r' || buffer[i] == '\n')) {
        i++;
    }
    // 解析第二个数字
    *b = 0;
    while (i < length && buffer[i] >= '0' && buffer[i] <= '9') {
        *b = (*b) * 10 + buffer[i] - '0';
        i++;
    }
    // 跳过空格和换行符
    while (i < length && (buffer[i] == ' ' || buffer[i] == '\r' || buffer[i] == '\n')) {
        i++;
    }
    // 解析第三个数字
    *c = 0;
    while (i < length && buffer[i] >= '0' && buffer[i] <= '9') {
        *c = (*c) * 10 + buffer[i] - '0';
        i++;
    }
}
/* USER CODE END 1 */
