/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include <math.h>
#include "bmi08x.h"
#include "bmi088.h"
#include "imu.h"

#define LED_PIN 	GPIO_PIN_6  
#define LED_PORT 	GPIOA  
#define A 			0.007177734375f
#define B 			0.00053263221801584764920766930190693f

void SystemClock_Config(void);
int8_t stm32_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t stm32_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)	//定时器2中断服务函数
{
	if(htim == &htim2)
	{
		 HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
	}
}

int fputc(int ch, FILE *f) //printf重定向
{
	uint8_t temp[1] = {ch};
	HAL_UART_Transmit(&huart1, temp, 1, 2);//
	return ch;
}

////////////////////////////////////////////定义的各类变量/////////////////////////////////////////////////

int8_t rslt;  //接受传感器各类状态的变量
uint8_t data = 0;

float accRange;
float gyro_x;
float gyro_y;
float gyro_z;
float accel_x;
float accel_y;
float accel_z;


struct bmi08x_sensor_data user_accel_bmi088;
struct bmi08x_sensor_data user_gyro_bmi088;

struct bmi08x_dev dev = {
        .accel_id = BMI08X_ACCEL_I2C_ADDR_SECONDARY,
        .gyro_id = BMI08X_GYRO_I2C_ADDR_SECONDARY,
        .intf = BMI08X_I2C_INTF,  
        .read = &stm32_i2c_read, 
        .write = &stm32_i2c_write, 
        .delay_ms = &HAL_Delay
};
////////////////////////////////////////////定义的各类变量/////////////////////////////////////////////////

int main(void)
{
	HAL_Init();
	SystemClock_Config();

	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_USART1_UART_Init();
	MX_TIM2_Init();

	HAL_TIM_Base_Start_IT(&htim2);	//开启定时器2基本功能
	
/////////////////////////////////////BMI088初始化相关//////////////////////////////////////////
	rslt = bmi088_init(&dev);
	
	if(rslt == BMI08X_OK) 
	{
		/* Read accel chip id */
		rslt = bmi08a_get_regs(BMI08X_ACCEL_CHIP_ID_REG, &data, 1, &dev);
		if(rslt == BMI08X_OK) 
		{
			/* Read gyro chip id */
			rslt = bmi08g_get_regs(BMI08X_GYRO_CHIP_ID_REG, &data, 1, &dev);
			printf("GYRO Initialization OK\n\r");
			printf("GYRO Chip ID: 0x%02X\n\r",data);
		}
	}
	else
	{
		printf("BMI088 Initialization Error\n\r");
		while(1);
	}
	
	/* Perform soft reset */
	rslt = bmi08a_soft_reset(&dev);
	if (rslt != BMI08X_OK)
	{
		printf("BMI088 Soft Reset Error\n\r");
		while(1);
	}
	/* Read the accel power mode */
	rslt = bmi08a_get_power_mode(&dev);
	/* Read the accel sensor config parameters (odr,bw,range) */
	rslt = bmi08a_get_meas_conf(&dev);
	/* Initialize the device instance as per the initialization example */

	/* Assign the desired configurations */
	dev.accel_cfg.bw = BMI08X_ACCEL_BW_NORMAL;
	dev.accel_cfg.odr = BMI08X_ACCEL_ODR_1600_HZ;
	dev.accel_cfg.range = BMI088_ACCEL_RANGE_24G;
	dev.accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE;
	
	rslt = bmi08a_set_power_mode(&dev);
	
	/* Wait for 10ms to switch between the power modes - delay taken care inside the function */
	rslt = bmi08a_set_meas_conf(&dev);
		
	/* Configuring the gyro	 */
	dev.gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;
	
	rslt = bmi08g_set_power_mode(&dev);
	/* Wait for 30ms to switch between the power modes - delay taken care inside the function */
	
	/* Assign the desired configurations */
	dev.gyro_cfg.odr = BMI08X_GYRO_BW_23_ODR_200_HZ;
	dev.gyro_cfg.range = BMI08X_GYRO_RANGE_1000_DPS;
	dev.gyro_cfg.bw = BMI08X_GYRO_BW_23_ODR_200_HZ;
	
	rslt = bmi08g_set_meas_conf(&dev);
	   
//	switch(dev.accel_cfg.range)
//	{
//		case BMI088_ACCEL_RANGE_3G:  accRange = 3000;  break;
//		case BMI088_ACCEL_RANGE_6G:  accRange = 6000;  break;
//		case BMI088_ACCEL_RANGE_12G: accRange = 12000; break;
//		case BMI088_ACCEL_RANGE_24G: accRange = 24000; break;
//		default: accRange = 24000;		
//	}
	
	
  while (1)
  {
	rslt = bmi08a_get_data(&user_accel_bmi088, &dev);
	/* Read the sensor data into the sensor data instance */
	rslt = bmi08g_get_data(&user_gyro_bmi088, &dev);
	  
	gyro_x = user_gyro_bmi088.x;
	gyro_y = user_gyro_bmi088.y;
	gyro_z = user_gyro_bmi088.z;
	accel_x = user_accel_bmi088.x;
	accel_y = user_accel_bmi088.y;
	accel_z = user_accel_bmi088.z;
	
	printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n\r",accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z);
	
	HAL_Delay(10);
  }
}



int8_t stm32_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	HAL_I2C_Mem_Write(&hi2c1, dev_addr<<1, reg_addr, 1, data, len, 100);
	while(HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_BUSY);
	
	return 0;
}

int8_t stm32_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{	
	HAL_I2C_Mem_Read(&hi2c1, dev_addr<<1, reg_addr, 1, data, len, 100);

	return 0;
}




























/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
