#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "position.h"

/***************************************各类函数声明****************************************/
void SystemClock_Config(void);
int8_t stm32_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t stm32_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
void BMI088_InitFuc(void);
void MPU9250_InitFuc(void);

/*****************************各类变量定义**********************************/

int8_t IMU_Res;//记录IMU初始化状态
uint8_t data = 0;

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

volatile Vector3 V3 = {0, 0, 0};
int t = 0;			//t用来控制串口输出频率	改为100ms输出一次
u8 device_id;
u8 MPU_Res;

/*****************************主函数*******************************/

int main(void)
{
	HAL_Init();
	SystemClock_Config();
	delay_init(168);
	
	MX_GPIO_Init();
	MX_I2C1_Init();
	SoftSim_IIC_Init();
	//其他相关函数的参数初始化
	LPF2_ParamSet(50, 8);
	pos_Estimate_Init();

	MX_USART1_UART_Init();
	MX_TIM2_Init();
	HAL_TIM_Base_Start_IT(&htim2);
	
/**********************************BMI088初始化相关*******************************************/
	BMI088_InitFuc();	
/***************************MPU9250初始化相关*******************************************/	
	MPU9250_InitFuc();
	
	while(1)
	{
		MPU_Res = mpu_mpl_get_data(&Pitch_9AX,&Roll_9AX,&Yaw_9AX);
		Temp_9AX = MPU_Get_Temperature();//得到温度值（扩大了100倍）
		if(MPU_Res == 0)
			printf("%.2f,%.2f,%.2f,%d\r\n",Pitch_9AX,Roll_9AX,Yaw_9AX,Temp_9AX); 
 		
		delay_ms(10);
	}
	
	while (1)//位置估计 + BMI088角度解算
	{	   
	//printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n\r",accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z);//串口输出原始数据 		
		Pos_Estimate(gyro_x, gyro_y, gyro_z, V3.x, V3.y, V3.z);
	//printf("%.2f,%.2f,%.2f\r\n",-Pitch,Roll,Yaw);
	}
  
  
  
}

/*************************TIM2 中断回调函数 5ms一次******************************/

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)	
{
	static int count = 0;
	static int count_delay = 0;
	
	#ifdef POSITION_CALC
		static float gyro_bias_z;
		static double gyro_ave_bias_z;
		static int count1;
		static char Yaw_turn = 0;
	#endif
	if(htim == &htim2)
	{	
		//等待一段时间，让陀螺仪数据稳定
		if(count_delay < 1000)
		{
			count_delay++;
			return;
		}
		//读取加速度计数据
		IMU_Res = bmi08a_get_data(&user_accel_bmi088, &dev);

		//对加速度计数据进行窗口滤波
		ACC_XYZ_Window_Filter(&user_accel_bmi088);

		V3.x = user_accel_bmi088.x*A;
		V3.y = user_accel_bmi088.y*A;
		V3.z = user_accel_bmi088.z*A;

		//读取陀螺仪数据
		IMU_Res = bmi08g_get_data(&user_gyro_bmi088, &dev);
		gyro_x = user_gyro_bmi088.x*B;
		gyro_y = user_gyro_bmi088.y*B;
		gyro_z = user_gyro_bmi088.z*B;
				
		#ifdef POSITION_CALC
			gyro_z1 = (double)gyro_z * R2D;
			
			//对陀螺仪z轴数据进行窗口滤波
			gyro_z1 = Single_Window_Filter(gyro_z1);

			//对陀螺仪数据进行二阶低通滤波
			gyro_z1 = LPF2_Calculate(gyro_z1);
			
			//对陀螺仪z轴数据进行零偏消除
			if(fabs(gyro_z1) < THRESHOLD)
			{
				count1++;
				gyro_bias_z += gyro_z1;
				if(count1 > 5)
				{
					gyro_ave_bias_z = gyro_bias_z/5;
					count1 = 0;
					gyro_bias_z = 0;	
				}	
				gyro_z1 -= gyro_ave_bias_z*0.835; 
			}
			//gyro_z1 -= gyro_ave_bias_z*0.835; 
					
			//去除零偏之后积分出角度
			Yaw += (double)gyro_z1*0.005;
			//将角度限制在±180度之间
			if(Yaw > 180)
			{
				Yaw -= 360;
				Yaw_turn++;	//可以记录转过的圈数
			}
			else if(Yaw < -180)
			{
				Yaw += 360;
				Yaw_turn--;
			}
		#endif		
		t++;
		if(t >= 20)
		{
			//printf("%f,%f,%f\r\n",-Pitch,Roll,Yaw);  
			t = 0;
		}
			
		count++;
		if(count == 50)
		{
			HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
			count = 0;
		}
	}
}


//printf重定向
int fputc(int ch, FILE *f) 
{
	uint8_t temp[1] = {ch};
	HAL_UART_Transmit(&huart1, temp, 1, 2);//
	return ch;
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





/*******************************BMI088初始化相关*******************************************/
void BMI088_InitFuc(void)
{
	IMU_Res = bmi088_init(&dev);	
	if(IMU_Res == BMI08X_OK) 
	{
		IMU_Res = bmi08a_get_regs(BMI08X_ACCEL_CHIP_ID_REG, &data, 1, &dev);
		if(IMU_Res == BMI08X_OK) 
		{
			IMU_Res = bmi08g_get_regs(BMI08X_GYRO_CHIP_ID_REG, &data, 1, &dev);
		}
	}
	else
	{
		printf("BMI088 Initial Not OK!!!\r\n");
		while(1);
	}
	IMU_Res = bmi08a_soft_reset(&dev);
	if (IMU_Res != BMI08X_OK)
	{
		printf("BMI088 RESET Not OK!!!\r\n");
		while(1);
	}
	IMU_Res = bmi08a_get_power_mode(&dev);
	IMU_Res = bmi08a_get_meas_conf(&dev);
	dev.accel_cfg.bw = BMI08X_ACCEL_BW_NORMAL;
	dev.accel_cfg.odr = BMI08X_ACCEL_ODR_200_HZ;
	dev.accel_cfg.range = BMI088_ACCEL_RANGE_24G;
	dev.accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE;
	IMU_Res = bmi08a_set_power_mode(&dev);
	IMU_Res = bmi08a_set_meas_conf(&dev);
	dev.gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;	
	IMU_Res = bmi08g_set_power_mode(&dev);
	dev.gyro_cfg.odr = BMI08X_GYRO_BW_23_ODR_200_HZ;
	dev.gyro_cfg.range = BMI08X_GYRO_RANGE_1000_DPS;
	dev.gyro_cfg.bw = BMI08X_GYRO_BW_23_ODR_200_HZ;	
	IMU_Res = bmi08g_set_meas_conf(&dev);
	printf("BMI088 Initial OK!!!\r\n");
}



void MPU9250_InitFuc(void)
{
	MPU_Res = MPU_Read_Len(MPU9250_ADDR, MPU_DEVICE_ID_REG, 1, &device_id);
	if(MPU_Res) 
	{
		printf("1\r\n");
		while(1);
	}
	if(device_id != MPU6500_ID) 
	{
		printf("2\r\n");
		while(1);
	}
	MPU_Res = mpu_dmp_init();
	if(MPU_Res) 
	{
		printf("%d\r\n",MPU_Res);
		while(1);
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
