#include <math.h>
#include "position.h"
#include "usart.h"
#include <stdio.h>
/************************用于实现设计滤波器的相关代码***************************/

// 定义滤波器系数结构体
typedef struct 
{
    double a[2];
    double b[2];
    double prev_input;    // 保存前一个输入值
    double prev_output;   // 保存前一个输出值
}BW_Filter;

BW_Filter H_filter;
BW_Filter L_filter;

// 初始化高通滤波器
void initHighPassFilter(BW_Filter *coeffs, double filtCutOff, double sampleRate) 
{
    double omega = 2.0 * M_PI_F * filtCutOff / sampleRate;
    double tan_omega = tan(omega / 2.0);

    // Calculate filter coefficients

    double gain = 1.0 / (1.0 + tan_omega);
    coeffs->a[0] = (tan_omega - 1) * gain;
    coeffs->b[0] = gain;
    coeffs->b[1] = -gain;
}

// 处理单个样本
float processHighPassFilter(BW_Filter *coeffs, float input) 
{
    // Apply the difference equation for the high-pass filter
    float output = coeffs->b[0]*input + coeffs->b[1]*(coeffs->prev_input) - coeffs->a[0]*(coeffs->prev_output);

    // 更新历史输入和输出
    coeffs->prev_input = input;
    coeffs->prev_output = output;

    return output;
}


void initLowPassFilter(BW_Filter *coeffs, double cutoffFreq, double sampleRate) 
{
    double tanTerm = tan(M_PI_F * cutoffFreq / sampleRate);
    
    coeffs->b[0] = tanTerm / (1.0 + tanTerm);
    coeffs->b[1] = coeffs->b[0];

    coeffs->a[0] = 1.0;
    coeffs->a[1] = (tanTerm - 1.0) / (tanTerm + 1.0);

    // 初始化前一个输入和输出
    coeffs->prev_input = 0.0;
    coeffs->prev_output = 0.0;
}

float processLowPassFilter(BW_Filter *coeffs,float input) 
{
    float output;
    // 计算当前输出
    output = coeffs->b[0] * input 
				+ coeffs->b[1] * coeffs->prev_input 
					- coeffs->a[1] * coeffs->prev_output;

    // 更新历史输入和输出
    coeffs->prev_input = input;
    coeffs->prev_output = output;

    return output;
}

/************************上述用于实现设计滤波器的相关代码***************************/



void q_conj(Quaternion *quart);
 

void Pos_Filter_Init(void)
{
    //初始化高通滤波器参数
    initHighPassFilter(&H_filter, FILTCUTOFF1, SAMPLE_FREQ);

    //初始化低通滤波器参数
    initLowPassFilter(&L_filter, FILTCUTOFF2, SAMPLE_FREQ);
}

Vector3 position_xyz;

//位置估计函数
void Pos_Estimate(float gx, float gy, float gz, float ax, float ay, float az)
{
    static float count_time = 0;

    static Vector3 rot_acc;
    static Vector3 speed_xyz;
    static Vector3 speed_xyz_last;
    static Vector3 speed_xyz_drift;
    static Vector3 speed_xyz_drift_last;

    float acc_module;

    Flag stationary = 0;  //静止标志位 1:静止 0:运动

    //计算加速度模值
    acc_module = sqrt(V3.x * V3.x + V3.y * V3.y + V3.z * V3.z);

    //对加速度计数据进行高通滤波
//    acc_module = processHighPassFilter(&H_filter, acc_module);

    //对数据取绝对值
    acc_module = fabs(acc_module);
    //对加速幅值进行低通滤波，以减少高频噪声。
    acc_module = processLowPassFilter(&L_filter, acc_module);

	//进行阈值检测，判断是否为静止状态
	if(fabs(gx) < STATIONARY_THRESHOLD && fabs(gy) < STATIONARY_THRESHOLD && fabs(gz) < STATIONARY_THRESHOLD)
        stationary = 1;
    else
        stationary = 0;

    //进行姿态解算，只用来获取四元数
    AHRS(gyro_x, gyro_y, gyro_z, V3.x, V3.y, V3.z);//四元数计算基本没问题
//	printf("%f %f %f %f", quart.q0, quart.q1, quart.q2, quart.q3);

    //将四元数从载体坐标系旋转到世界坐标系
    rot_acc = rotate_vector_by_quaternion(V3, quart);   //旋转后 载体->世界 

    //去除重力加速度   
    rot_acc.z -= GRAVITY; 
    
//	printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n", rot_acc.x, rot_acc.y, rot_acc.z,gyro_x, gyro_y, gyro_z);

    //速度计算
    //速度 = 速度 + 加速度 * 时间间隔  单位：m/s
    if(stationary == 0)
    {
        speed_xyz.x = speed_xyz_last.x + rot_acc.x * SAMPLE_TIME;
        speed_xyz.y = speed_xyz_last.y + rot_acc.y * SAMPLE_TIME;
        speed_xyz.z = speed_xyz_last.z + rot_acc.z * SAMPLE_TIME;
    }
    else //静止状态速度为0
    {
        speed_xyz = (Vector3){0, 0, 0};
    }
//    printf("%f,%f,%f\r\n", speed_xyz.x, speed_xyz.y, speed_xyz.z);

   // 对于非静止状态下的速度，计算漂移,把它从速度中移除
    if (stationary == 0) 
    {
        count_time++;
        if(count_time < 1) 
        {
            speed_xyz_drift_last.x = speed_xyz.x;
            speed_xyz_drift_last.y = speed_xyz.y;
            speed_xyz_drift_last.z = speed_xyz.z;
        }
        if(count_time >= 10)   //每50ms更新一次 速度漂移率 = 速度差 / 时间
        {
            speed_xyz_drift.x = (speed_xyz.x - speed_xyz_drift_last.x)/10;
            speed_xyz_drift.y = (speed_xyz.y - speed_xyz_drift_last.y)/10;
            speed_xyz_drift.z = (speed_xyz.z - speed_xyz_drift_last.z)/10;

            count_time = 0;
        }
    }
    //去除速度漂移(需要区分正负)
    // if(rot_acc.x > 0)
    //     speed_xyz.x += speed_xyz_drift.x * SAMPLE_TIME*1.51f;
    // else
    //     speed_xyz.x -= speed_xyz_drift.x * SAMPLE_TIME*1.5f;

    // if(rot_acc.y > 0)
    //     speed_xyz.y += speed_xyz_drift.y * SAMPLE_TIME*1.51f;
    // else
    //     speed_xyz.y -= speed_xyz_drift.y * SAMPLE_TIME*1.5f;

    // if(rot_acc.z > 0)
    //     speed_xyz.z += speed_xyz_drift.z * SAMPLE_TIME*1.51f;
    // else
    //     speed_xyz.z -= speed_xyz_drift.z * SAMPLE_TIME*1.5f;
		

	//位置计算   单位：m
    position_xyz.x += speed_xyz.x * SAMPLE_TIME;
    position_xyz.y += speed_xyz.y * SAMPLE_TIME;
    position_xyz.z += speed_xyz.z * SAMPLE_TIME;

//	printf("%f,%f,%f\r\n",position_xyz.x*100, position_xyz.y*100, position_xyz.z*100); //    单位：cm  

//    printf("%f,%f,%f\r\n",rot_acc.y, speed_xyz.y, position_xyz.y*100); 

    //更新速度
    speed_xyz_last = speed_xyz;  

}

// 共轭四元数用于将向量旋转到原点的逆方向   
Quaternion quat_conj(Quaternion quart) 
{
    Quaternion quart_conj;
    quart_conj.q0 =  quart.q0;
    quart_conj.q1 = -quart.q1;
    quart_conj.q2 = -quart.q2;
    quart_conj.q3 = -quart.q3;

    return quart_conj;
}

// 四元数与向量相乘
Quaternion quat_multiply(Quaternion quart1, Quaternion quart2) 
{
    Quaternion quart;
    quart.q0 = quart1.q0*quart2.q0 - quart1.q1*quart2.q1 - quart1.q2*quart2.q2 - quart1.q3*quart2.q3;
    quart.q1 = quart1.q0*quart2.q1 + quart1.q1*quart2.q0 + quart1.q2*quart2.q3 - quart1.q3*quart2.q2;
    quart.q2 = quart1.q0*quart2.q2 - quart1.q1*quart2.q3 + quart1.q2*quart2.q0 + quart1.q3*quart2.q1;
    quart.q3 = quart1.q0*quart2.q3 + quart1.q1*quart2.q2 - quart1.q2*quart2.q1 + quart1.q3*quart2.q0;
    return quart;
}


//使用四元数旋转向量
Vector3 rotate_vector_by_quaternion(Vector3 v3, Quaternion quart) 
{
    // 将三维向量转换到四维
    Quaternion q_v = {0, v3.x, v3.y, v3.z};     
    // 共轭四元数
    Quaternion q_conj = quat_conj(quart);
    // 矩阵相乘 v′ = q⋅v⋅q* 
    Quaternion temp = quat_multiply(quart, q_v); //首先计算 q * v  
    Quaternion q_result = quat_multiply(temp, q_conj);// 然后计算 (q * v_q) * q*

    // 返回旋转后的向量
    Vector3 v_rotated;
    v_rotated.x = q_result.q1;
    v_rotated.y = q_result.q2;
    v_rotated.z = q_result.q3;

    return v_rotated;
} 










/*****************用于实现加速度计数据保存的队列***************************/
//typedef struct {
//    float buffer[QUEUE_SIZE];  // 用于存储数据的缓冲区
//    int front;                 // 指向队列头部的指针
//    int rear;                  // 指向队列尾部的指针
//    int count;                 // 队列中的元素数量
//} Queue;    //创建一个循环队列

//Queue queue_am;

//// 初始化队列
//void initQueue(Queue* q) 
//{
//    q->front = 0;
//    q->rear = -1;
//    q->count = 0;
//}

//int isQueueEmpty(Queue* q) 
//{
//    return q->count == 0;
//}

//int isQueueFull(Queue* q) 
//{
//    return q->count == QUEUE_SIZE;
//}

//void push(Queue* q, float value) 
//{
//    if (isQueueFull(q)) {
//        // 队列已满，覆盖最早的元素
//        q->front = (q->front + 1) % QUEUE_SIZE;
//    } 
//    else
//        q->count++;

//    q->rear = (q->rear + 1) % QUEUE_SIZE;
//    q->buffer[q->rear] = value;
//}

//float pop(Queue* q) 
//{
//    if (isQueueEmpty(q)) 
//    {
//        printf("Queue is empty!\n");
//        return -1; // 或其他适当的错误值
//    } 
//    else 
//    {
//        float value = q->buffer[q->front];
//        q->front = (q->front + 1) % QUEUE_SIZE;
//        q->count--;
//        return value;
//    }
//}
/************************上述用于实现加速度计数据保存的队列***************************/


