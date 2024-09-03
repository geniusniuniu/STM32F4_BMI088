/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

    GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7; //PA6，7
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  //推挽输出
    GPIO_InitStruct.Pull = GPIO_PULLUP;          //上拉
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;     //高速
    HAL_GPIO_Init(GPIOA,&GPIO_InitStruct);
	
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);	//PA6置1 ,灭
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);	//PA7置1 ,灭

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
