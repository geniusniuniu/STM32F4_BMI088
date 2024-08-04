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

/***************************************各类函数声明****************************************/
void SystemClock_Config(void);
int8_t stm32_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t stm32_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
static float invSqrt(float x); 		//快速计算 1/Sqrt(x)
void AHRS(float gx, float gy, float gz, float ax, float ay, float az); //四元数姿态解算

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)	//定时器2中断回调函数500ms一次
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

////////////////////////////////////////////各类变量定义/////////////////////////////////////////////////

int8_t rslt;    //用来记录IMU初始化状态
uint8_t data = 0;


float gyro_x;
float gyro_y;
float gyro_z;
float accel_x;
float accel_y;
float accel_z;

float Pitch;
float Roll;
float Yaw;


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
////////////////////////////////////////////主函数/////////////////////////////////////////////////

int main(void)
{
	HAL_Init();
	SystemClock_Config();

	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_USART1_UART_Init();
	MX_TIM2_Init();

	HAL_TIM_Base_Start_IT(&htim2);
	
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
	   
/////////////////////////////////////BMI088初始化相关//////////////////////////////////////////
	
  while (1)
  {
    //读取加速度计和陀螺仪数据
    rslt = bmi08a_get_data(&user_accel_bmi088, &dev);
    rslt = bmi08g_get_data(&user_gyro_bmi088, &dev);
      
    gyro_x = user_gyro_bmi088.x*B;
		gyro_y = user_gyro_bmi088.y*B;
		gyro_z = user_gyro_bmi088.z*B;
		accel_x = user_accel_bmi088.x*A;
		accel_y = user_accel_bmi088.y*A;
		accel_z = user_accel_bmi088.z*A;
    //串口输出原始数据    
//    printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n\r",accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z);
    AHRS(gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z);
    printf("%.2f,%.2f,%.2f\n\r",Pitch,Roll,Yaw);
    HAL_Delay(10);
  }
}

/********************************I2C相关函数*********************************************/
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

// IMU 相关宏定义
#define	_IMU_PI		  3.14159265f
#define	_IMU_Kp		  0.8f
#define	_IMU_Ki		  0.001f
#define	_IMU_Half_T	0.005f         //表示四元数更新周期的一半

float  q0 = 1, q1 = 0, q2 = 0, q3 = 0;  //四元数
float  exInt = 0, eyInt = 0, ezInt = 0; //叉积计算误差的累计积分

void AHRS(float gx, float gy, float gz, float ax, float ay, float az) //四元数姿态解算
{
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;

    //归一化加速度计的三轴数据
    norm = invSqrt(ax*ax + ay*ay + az*az);
    ax = ax * norm;
	  ay = ay * norm;
	  az = az * norm;

    //加速度计重力向量转换到b系（四元数推出的实际重力方向）
    vx = 2 * (q1 * q3 - q0 * q2);
    vy = 2 * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3 ;
/**************************仅依靠加速计补偿无法修正Z轴的偏差*******************/
/**************************此处还需要通过磁力计来修正Z轴*******************/
    //叉积误差
    ex = (ay * vz - az * vy) ;
    ey = (az * vx - ax * vz) ;
    ez = (ax * vy - ay * vx) ;

    //叉积误差积分为角速度
    exInt = exInt + ex * _IMU_Ki;
    eyInt = eyInt + ey * _IMU_Ki;
    ezInt = ezInt + ez * _IMU_Ki;

    //角速度补偿
    gx = gx + _IMU_Kp * ex + exInt;
    gy = gy + _IMU_Kp * ey + eyInt;
    gz = gz + _IMU_Kp * ez + ezInt;

    //一阶龙格库塔法更新四元数
    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * _IMU_Half_T;
    q1 = q1 + ( q0 * gx + q2 * gz - q3 * gy) * _IMU_Half_T;
    q2 = q2 + ( q0 * gy - q1 * gz + q3 * gx) * _IMU_Half_T;
    q3 = q3 + ( q0 * gz + q1 * gy - q2 * gx) * _IMU_Half_T;

    //单位化四元数
    norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  	q0 = q0 * norm;
  	q1 = q1 * norm;
  	q2 = q2 * norm;  
  	q3 = q3 * norm;

    //四元数反解欧拉角
	  Yaw = atan2(2.f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3)* 57.3f;
	  Pitch = -asin(2.f * (q1 * q3 - q0 * q2))* 57.3f;
	  Roll = atan2(2.f * q2 * q3 + 2.f * q0 * q1, q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3)* 57.3f;

}

static float invSqrt(float x) 		//快速计算 1/Sqrt(x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
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
