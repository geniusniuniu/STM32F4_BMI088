#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "position.h"

#include "vl53l0x.h"
#include "vl53l0x_platform.h"

/***************************************各类函数声明****************************************/
void Init_All(void);
void BMI088_InitFunc(void);
void MPU9250_DMP_InitFunc(void);
void VL53L0x_InitFunc(void);
void vl53l0x_test(void);
void SystemClock_Config(void);
uint16_t* Filter_Window(uint16_t* Dis);
/***************************************各类变量定义****************************************/

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


float Pitch_9AX,Roll_9AX,Yaw_9AX;	//欧拉角
short Temp_9AX;                     //温度



volatile Vector3 V3 = {0, 0, 0};
int t = 0;			//t用来控制串口输出频率	改为100ms输出一次
u8 device_id;
u8 MPU_Res;

/*************************************** main函数 ****************************************/

int main(void)
{
	Init_All();		
/************************************** IMU初始化 ***************************************/
	BMI088_InitFunc();		
	MPU9250_DMP_InitFunc();	
/************************************** TOF初始化 ***************************************/	
	VL53L0x_InitFunc();
	while (1)//位置估计 + BMI088角度解算
	{	   
		//printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n\r",accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z);//串口输出原始数据 	
		//printf("%.2f,%.2f,%.2f\r\n",-Pitch,Roll,Yaw);		
		
		
		mpu_mpl_get_data(&Pitch_9AX,&Roll_9AX,&Yaw_9AX);
		Pos_Estimate(gyro_x, gyro_y, gyro_z, V3.x, V3.y, V3.z);
		//HAL_Delay(100);
		//vl53l0x_test();
		//Temp_9AX = MPU_Get_Temperature();//得到MPU9250的温度值（扩大了100倍）
	} 
  
}



/****************************** TIM2 中断回调函数 5ms一次 ********************************/

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
				//gyro_z1 -= gyro_ave_bias_z*0.835; 
			}
			gyro_z1 -= gyro_ave_bias_z*0.835; 
					
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
			printf("6Axis:%f,%f,%f\r\n",-Pitch,Roll,Yaw); 
			printf("9Axis:%.2f,%.2f,%.2f\r\n",Pitch_9AX,Roll_9AX,Yaw_9AX); 
			
			t = 0;
		}
			
		count++;
		if(count == 100)
		{
			HAL_GPIO_TogglePin(LED_PORT, LED_PIN_1);
			count = 0;
		}
	}
}


/***********************************BMI088初始化相关***************************************/
void BMI088_InitFunc(void)
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



void MPU9250_DMP_InitFunc(void)
{
	MPU_Res = MPU_Read_Len(MPU9250_ADDR, MPU_DEVICE_ID_REG, 1, &device_id);	// 判断IIC实物接线是否有问题 															
	if(MPU_Res)																// 同时需要判断MPU9250		
	{																		// AD0引脚是否接错
		printf("Read Error:%d\r\n",MPU_Res);
		while(1);
	}
	
	MPU_Res = mpu_dmp_init();
	while(1)
	{
		HAL_GPIO_TogglePin(LED_PORT, LED_PIN_2);
		if(MPU_Res)		
		{
			printf("DMP Error:%d\r\n",MPU_Res);			
		}
		else
			break;
		HAL_Delay(100);
	}
	printf("MPU_DMP Init Succeed\r\n");
}


void VL53L0x_InitFunc(void)
{	
	//使用片选信号启动第一个tof
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET  );	
	HAL_Delay(100);	//等待，确保tof启动
	while(vl53l0x_init(&vl53l0x_dev[Axis_X],Xshut_Pin_X))//使用默认地址初始化第一个tof
	{
		printf("Xaxis_VL53L0x Error!!!\n\r");
		HAL_Delay(100);
		LED0=!LED0;//DS0闪烁
	}
	printf("Xaxis_VL53L0X Init OK\r\n");	
	vl53l0x_Addr_set(&vl53l0x_dev[Axis_X],TOF_X_ADDR);
	printf("Addr:%#x\r\n",vl53l0x_dev[Axis_X].I2cDevAddr);	
	
	//启动第二个TOF
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
	HAL_Delay(100);	//等待，确保tof启动
	
	//使用默认地址初始化第二个tof
	while(vl53l0x_init(&vl53l0x_dev[Axis_Y],Xshut_Pin_Y))//vl53l0x初始化
	{
		printf("Yaxis_VL53L0x Error!!!\n\r");
		HAL_Delay(100);
		LED0=!LED0;//DS0闪烁
	}
	printf("Yaxis_VL53L0X Init OK\r\n");	
	vl53l0x_Addr_set(&vl53l0x_dev[Axis_Y],TOF_Y_ADDR);
	printf("Addr:%#x\r\n",vl53l0x_dev[Axis_Y].I2cDevAddr);
	

//	//启动第三个TOF
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
	HAL_Delay(100);	//等待，确保tof启动	
	
	//使用默认地址初始化第三个tof
	while(vl53l0x_init(&vl53l0x_dev[Axis_Z],Xshut_Pin_Z))//vl53l0x初始化
	{
		printf("Zaxis_VL53L0x Error!!!\n\r");
		HAL_Delay(100);
		LED0=!LED0;//DS0闪烁
	}
	printf("Zaxis_VL53L0X Init OK\r\n");
	
	//修改第三个tof的iic操作地址
	vl53l0x_Addr_set(&vl53l0x_dev[Axis_Z],TOF_Z_ADDR);
	printf("Addr:%#x\r\n",vl53l0x_dev[Axis_Z].I2cDevAddr);
	
	//修改TOF读取模式，现在是默认模式，可以在宏定义中找到别的模式
	if(VL53L0X_ERROR_NONE == vl53l0x_set_mode(&vl53l0x_dev[Axis_X],Default_Mode)) 
		printf("Xaxis_VL53L0X MODE SET OK\r\n");
	
	if(VL53L0X_ERROR_NONE == vl53l0x_set_mode(&vl53l0x_dev[Axis_Y],Default_Mode)) 
		printf("Yaxis_VL53L0X MODE SET OK\r\n");
	
	if(VL53L0X_ERROR_NONE == vl53l0x_set_mode(&vl53l0x_dev[Axis_Z],Default_Mode)) 
		printf("Zaxis_VL53L0X MODE SET OK\r\n");

}

//VL53L0X测试程序
void vl53l0x_test(void)
{   
	u8 i=0;
	VL53L0X_Error status = 0; 
	static char buf[VL53L0X_MAX_STRING_LENGTH];//测试模式字符串字符缓冲区
	uint16_t* result = NULL;
		  	 
	status = vl53l0x_start_single_test(&vl53l0x_dev[Axis_X],Axis_X,&vl53l0x_data[Axis_X],buf);
	status = vl53l0x_start_single_test(&vl53l0x_dev[Axis_Y],Axis_Y,&vl53l0x_data[Axis_Y],buf);
	status = vl53l0x_start_single_test(&vl53l0x_dev[Axis_Z],Axis_Z,&vl53l0x_data[Axis_Z],buf);
	result = Filter_Window(Distance_data);
	 
	if(status == VL53L0X_ERROR_NONE)
	{
		printf("%d,%d,%d\r\n",Distance_data[Axis_X],Distance_data[Axis_Y],Distance_data[Axis_Z]);
	}
	else
		printf("Status:%d\r\n",status);	
	
	i++;
	if(i==5)	//闪灯
	{			 
		LED1 =! LED1;
		i = 0;
	} 
}

void Init_All(void)
{
	//系统时钟配置初始化
	HAL_Init();
	SystemClock_Config();
	delay_init(168);
	
	//IO引脚初始化
	MX_GPIO_Init();
	XShut_PinInit();
	
	//初始化三个不同的IIC接口
	MX_I2C1_Init();
	SoftSim_IIC_Init();
	VL53L0X_i2c_init();	
	
	//外设初始化
	MX_USART1_UART_Init();
	MX_TIM2_Init();
	HAL_TIM_Base_Start_IT(&htim2);
	
	//相关函数的参数初始化
	LPF2_ParamSet(50, 8);
	Pos_Filter_Init();
}

uint16_t* Filter_Window(uint16_t* Dis) 
{
    static uint16_t filter_buf[3][FILTER_N + 1] = {0};
    int i = 0,j = 0;    
    uint16_t filter_sum[3] = {0};
    filter_buf[0][FILTER_N] = Dis[0];
	filter_buf[1][FILTER_N] = Dis[1];
	filter_buf[2][FILTER_N] = Dis[2];
	
    for(j = 0; j < 3;j++)
	{
		for(i = 0; i < FILTER_N; i++) 
		{
			filter_buf[j][i] = filter_buf[j][i + 1]; 	//�����������ƣ���λ�Ե�
			filter_sum[j] += filter_buf[j][i];
		}
		Dis[j] = (uint16_t)(filter_sum[j] / FILTER_N);
	}
    return Dis;
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
