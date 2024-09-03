#ifndef __POSITION_H
#define	__POSITION_H

#include "imu.h"
#include "main.h"

#define QUEUE_SIZE              10  // 队列的大小

//采样频率
#define SAMPLE_FREQ             200.0f
#define SAMPLE_TIME             1.0f/SAMPLE_FREQ

//设置高通滤波器截至频率
#define FILTCUTOFF1             0.001f
//设置低通截止频率为5 Hz
#define FILTCUTOFF2             5.0f

/*
    POSITION_CALC 1: 通过加速度计计算位置
    POSITION_CALC 0: 通过四元数计算角度
*/
#define POSITION_CALC             

//判断是否静止的阈值
#define STATIONARY_THRESHOLD    0.02f

typedef char Flag;

extern Vector3 position_xyz;

Vector3 rotate_vector_by_quaternion(Vector3 v, Quaternion quart);

void Pos_Filter_Init(void);
// 位置估计函数
void Pos_Estimate(float gx, float gy, float gz, float ax, float ay, float az);

#endif



