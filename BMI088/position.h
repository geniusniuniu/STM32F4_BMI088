#ifndef __POSITION_H
#define	__POSITION_H

#include "imu.h"
#include "main.h"

#define QUEUE_SIZE              10  // 队列的最大大小

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

#endif



