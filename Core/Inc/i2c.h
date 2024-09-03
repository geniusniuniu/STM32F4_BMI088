/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.h
  * @brief   This file contains all the function prototypes for
  *          the i2c.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_I2C1_Init(void);
int8_t stm32_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t stm32_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */

