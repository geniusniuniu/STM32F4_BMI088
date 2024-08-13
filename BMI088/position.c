#include <math.h>
#include "position.h"
#include "usart.h"
#include <stdio.h>

/*****************用于实现加速度计数据保存的队列***************************/
typedef struct {
    float buffer[QUEUE_SIZE];  // 用于存储数据的缓冲区
    int front;                 // 指向队列头部的指针
    int rear;                  // 指向队列尾部的指针
    int count;                 // 队列中的元素数量
} Queue;    //创建一个循环队列

Queue queue_am;

// 初始化队列
void initQueue(Queue* q) 
{
    q->front = 0;
    q->rear = -1;
    q->count = 0;
}

int isQueueEmpty(Queue* q) 
{
    return q->count == 0;
}

int isQueueFull(Queue* q) 
{
    return q->count == QUEUE_SIZE;
}

void push(Queue* q, float value) 
{
    if (isQueueFull(q)) {
        // 队列已满，覆盖最早的元素
        q->front = (q->front + 1) % QUEUE_SIZE;
    } 
    else
        q->count++;

    q->rear = (q->rear + 1) % QUEUE_SIZE;
    q->buffer[q->rear] = value;
}

float pop(Queue* q) 
{
    if (isQueueEmpty(q)) 
    {
        printf("Queue is empty!\n");
        return -1; // 或其他适当的错误值
    } 
    else 
    {
        float value = q->buffer[q->front];
        q->front = (q->front + 1) % QUEUE_SIZE;
        q->count--;
        return value;
    }
}
/************************上述用于实现加速度计数据保存的队列***************************/


/************************用于实现设计滤波器的相关代码***************************/
typedef struct {
    double a0, a1, b1;  // 滤波器系数
    double z1;          // 滤波器的前一输入值
} Filter;

Filter H_filter;
Filter L_filter;

// 初始化高通滤波器
void initHighPassFilter(Filter* filter, double cutoffFreq, double sampleRate) 
{
    double theta = 2.0 * M_PI_F * cutoffFreq / sampleRate;
    double gamma = cos(theta) / (1.0 + sin(theta));

    filter->a0 = (1.0 + gamma) / 2.0;
    filter->a1 = -(1.0 + gamma) / 2.0;
    filter->b1 = gamma;
    filter->z1 = 0.0;
}

// 处理单个样本
float processHighPassFilter(Filter* filter, float input) 
{
    float output = filter->a0*input + filter->a1*filter->z1 - filter->b1*filter->z1;
    filter->z1 = input;
    return output;
}

// 初始化低通滤波器
void initLowPassFilter(Filter* filter, double cutoffFreq, double sampleRate) 
{
    double theta = 2.0 *  M_PI_F * cutoffFreq / sampleRate;
    double d = 1.0 / (1.0 + sin(theta));
    filter->a0 = (1.0 - cos(theta)) * d / 2.0;
    filter->a1 = (1.0 - cos(theta)) * d / 2.0;
    filter->b1 = (1.0 - d) * (-1.0);
    filter->z1 = 0.0;
}

// 处理单个样本
float processLowPassFilter(Filter* filter, float input) 
{
    float output = filter->a0 * input + filter->a1 * filter->z1 - filter->b1 * filter->z1;
    filter->z1 = output;
    return output;
}

/************************用于实现设计滤波器的相关代码***************************/

void q_conj(Quaternion *quart);
 

void pos_Estimate_Init()
{
    //设置高通滤波器截至频率
    float FiltCutoff1 =  0.001f;
    //设置低通截止频率为5 Hz
    float FiltCutoff2 =  5;
    //初始化队列
    initQueue(&queue_am);
    //初始化高通滤波器参数
    initHighPassFilter(&H_filter, FiltCutoff1, SAMPLE_FREQ);

    //初始化低通滤波器参数
    initLowPassFilter(&L_filter, FiltCutoff2, SAMPLE_FREQ);
}

Vector3 rotate_vector_by_quaternion(Vector3 v, Quaternion quart);

// 位置估计函数
float Pos_Estimate(float gx, float gy, float gz, float ax, float ay, float az)
{
    static float count_2s = 0;
    static float ax_mean = 0;
    static float ay_mean = 0;
    static float az_mean = 0;
    
    Vector3 Rotated_V3;

    float acc_module;
    float sum_ax = 0;
    float sum_ay = 0;
    float sum_az = 0;

    Flag stationary = 0;

    //计算加速度模值
    acc_module = sqrt(ax * ax + ay * ay + az * az);

    //butterworth高通滤波
    //对加速度计数据进行高通滤波
    acc_module = processHighPassFilter(&H_filter, acc_module);

    //对数据取绝对值
    acc_module = fabs(acc_module);

    //对加速幅值再次进行低通滤波，以减少高频噪声。
    acc_module = processLowPassFilter(&L_filter, acc_module);

    //进行阈值检测，判断是否为静止状态
    if(acc_module < STATIONARY_THRESHOLD)
    {
        stationary = 1;
    }

    //进行初始收敛，以获得IMU数据的姿态估计
    //稳定后的两秒的静态数据平均值作为后续的姿态估计的初始状态
    if(count_2s < 400)
    {
        sum_ax += ax;
        sum_ay += ay;
        sum_az += az;
        count_2s++;
    }
    else
    {
        //计算队列中的数据的平均值
        ax_mean = sum_ax / 400;
        ay_mean = sum_ay / 400;
        az_mean = sum_az / 400;

    }
    
    // if(stationary == 0) //在静止状态下，对加速度计的输入更加敏感
    // {
    //     imu_Kp = 0;
    // }
    // else               //在动态状态下不依赖于加速度计的输入，主要依靠陀螺仪的数据
    // {
    //     imu_Kp = 5.0;
    // }



    //进行姿态解算，只用来获取四元数，不对角度解算。
    AHRS(gx, gy, gz, ax, ay, az);

    Rotated_V3 = rotate_vector_by_quaternion(V3, quart);
    Rotated_V3.az -= 1;

    Rotated_V3.ax *= GRAVITY;
    Rotated_V3.ay *= GRAVITY;
    Rotated_V3.az *= GRAVITY;

    //速度计算
    //速度 = 速度 + 加速度 * 时间间隔

    return 0;
    
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
    Quaternion q;
    q.q0 = quart1.q0*quart2.q0 - quart1.q1*quart2.q1 - quart1.q2*quart2.q2 - quart1.q3*quart2.q3;
    q.q1 = quart1.q0*quart2.q1 + quart1.q1*quart2.q0 + quart1.q2*quart2.q3 - quart1.q3*quart2.q2;
    q.q2 = quart1.q0*quart2.q2 - quart1.q1*quart2.q3 + quart1.q2*quart2.q0 + quart1.q3*quart2.q1;
    q.q3 = quart1.q0*quart2.q3 + quart1.q1*quart2.q2 - quart1.q2*quart2.q1 + quart1.q3*quart2.q0;
    return q;
}


// 使用四元数旋转向量
Vector3 rotate_vector_by_quaternion(Vector3 v, Quaternion quart) 
{
    // 将向量转换为四元数，w = 0
    Quaternion q_v = {0, v.ax, v.ay, v.az};     
    // 共轭四元数
    Quaternion q_conj = quat_conj(quart);
    // 矩阵相乘 v′ = q⋅v⋅q* 
    Quaternion q_result = quat_multiply(quat_multiply(quart, q_v), q_conj);

    // 返回旋转后的向量
    Vector3 v_rotated;
    v_rotated.ax = q_result.q1;
    v_rotated.ay = q_result.q2;
    v_rotated.az = q_result.q3;

    return v_rotated;
} 






