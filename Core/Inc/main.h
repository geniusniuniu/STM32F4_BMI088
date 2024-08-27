#ifndef __MAIN_H
#define __MAIN_H

#include "sys.h"
#include "delay.h"

#include <stdio.h>
#include "math.h"

#include "bmi08x.h"
#include "bmi088.h"
#include "imu.h"

#include "myiic.h"
#include "mpu9250.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 



//程序运行状态指示灯PA6
#define LED_PIN 		GPIO_PIN_6  								
#define LED_PORT 		GPIOA  

#define ORIGIN_A		0.000732421875								//原始加速度计数据 （24/32768）
#define A 				0.007177734375f								//将原始加速度计数据转换成m/s^2
//加速度原始数据转换成m/s^2
#define GRAVITY         9.81f
#define B 				0.00053263221801584764920766930190693f		//原始陀螺仪数据转 单位（rad/s）
#define R2D 			180.0f/M_PI_F								//弧度转角度




#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

void Error_Handler(void);

typedef struct {
    float x;
    float y;
    float z;
} Vector3;


extern volatile Vector3 V3;

extern float Pitch_9AX,Roll_9AX,Yaw_9AX;	//欧拉角
extern short Temp_9AX;                     //温度

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
