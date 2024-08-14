#ifndef __POSITION_H
#define	__POSITION_H

#include "imu.h"
#include "main.h"

#define QUEUE_SIZE              10  // 队列的大小

#define SAMPLE_FREQ             200.0f
#define SAMPLE_TIME             1.0f/SAMPLE_FREQ

#define GRAVITY                 9.81f
/*
    POSITION_CALC 1: 通过加速度计计算位置
    POSITION_CALC 0: 通过四元数计算角度
*/
#define POSITION_CALC           1   

#define STATIONARY_THRESHOLD    0.1f

typedef char Flag;

Vector3 rotate_vector_by_quaternion(Vector3 v, Quaternion quart);

void pos_Estimate_Init(void);
// 位置估计函数
float Pos_Estimate(float gx, float gy, float gz, float ax, float ay, float az);

#endif



