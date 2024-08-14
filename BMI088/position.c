#include <math.h>
#include "position.h"
#include "usart.h"
#include <stdio.h>
/************************用于实现设计滤波器的相关代码***************************/
typedef struct 
{
    double a0, a1, b1;  // 滤波器系数
    double z1;          // 滤波器的前一输入值
}Filter;

Filter H_filter;

// 定义滤波器系数结构体
typedef struct 
{
    double a[2];
    double b[2];
    double prev_input;    // 保存前一个输入值
    double prev_output;   // 保存前一个输出值
}BW_Filter;

BW_Filter L_filter;

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
//void initLowPassFilter(Filter* filter, double cutoffFreq, double sampleRate) 
//{
//    double theta = 2.0 *  M_PI_F * cutoffFreq / sampleRate;
//    double d = 1.0 / (1.0 + sin(theta));
//    filter->a0 = (1.0 - cos(theta)) * d / 2.0;
//    filter->a1 = (1.0 - cos(theta)) * d / 2.0;
//    filter->b1 = (1.0 - d) * (-1.0);
//    filter->z1 = 0.0;
//}


// 处理单个样本
//float processLowPassFilter(Filter* filter, float input) 
//{
//    float output = filter->a0*input + filter->a1*filter->z1 - filter->b1*filter->z1;
//    filter->z1 = output;
//    return output;
//}


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




/************************用于实现设计滤波器的相关代码***************************/

void q_conj(Quaternion *quart);
 

void pos_Estimate_Init(void)
{
    //初始化高通滤波器参数
    initHighPassFilter(&H_filter, FILTCUTOFF1, SAMPLE_FREQ);

    //初始化低通滤波器参数
    initLowPassFilter(&L_filter, FILTCUTOFF2, SAMPLE_FREQ);
}


//位置估计函数
Vector3 Pos_Estimate(float gx, float gy, float gz, float ax, float ay, float az)
{
    // static float count_2s = 0;
    // static float ax_mean = 0;
    // static float ay_mean = 0;
    // static float az_mean = 0;
    // float sum_ax = 0;
    // float sum_ay = 0;
    // float sum_az = 0;
//    printf("START!!!\r\n");
    static float count_time = 0;

    Vector3 rot_acc;
    Vector3 speed_xyz;
    Vector3 speed_xyz_last;
    Vector3 speed_xyz_drift;
    Vector3 speed_xyz_drift_last;
    Vector3 position_xyz;

    float acc_module;

    Flag stationary = 0;  //静止标志位 1:静止 0:运动

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
/******************************************这块代码有问题******************************/
if(acc_module < STATIONARY_THRESHOLD) //acc_module一直是 1.01，应改为三轴加速度判断ax等
    {
        stationary = 1;
    }

	
	
    // //进行初始收敛，以获得IMU数据的姿态估计
    // //稳定后的两秒的静态数据平均值作为后续的姿态估计的初始状态
    // if(count_2s < 400)
    // {
    //     sum_ax += ax;
    //     sum_ay += ay;
    //     sum_az += az;
    //     count_2s++;
    // }
    // else
    // {
    //     //计算队列中的数据的平均值
    //     ax_mean = sum_ax / 400;
    //     ay_mean = sum_ay / 400;
    //     az_mean = sum_az / 400;

    // }
    
    if(stationary == 0) //在静止状态下，对加速度计的输入更加敏感
    {
        imu_Kp = 0;
    }
    else               //在动态状态下不依赖于加速度计的输入，主要依靠陀螺仪的数据
    {
        imu_Kp = 5.1;
    }

    //进行姿态解算，只用来获取四元数
    AHRS(gx, gy, gz, ax, ay, az);
//	printf("%f,%f,%f,%f\r\n", quart.q0, quart.q1, quart.q2, quart.q3);
    rot_acc = rotate_vector_by_quaternion(V3, quart);
    rot_acc.z -= 1;

    rot_acc.x *= GRAVITY;
    rot_acc.y *= GRAVITY;
    rot_acc.z *= GRAVITY;
//	printf("%f,%f,%f\r\n", rot_acc.x, rot_acc.y, rot_acc.z);
    //速度计算
    //速度 = 速度 + 加速度 * 时间间隔
    if(stationary == 0)
    {
        speed_xyz.x = speed_xyz_last.x + rot_acc.x * SAMPLE_TIME;
        speed_xyz.y = speed_xyz_last.y + rot_acc.y * SAMPLE_TIME;
        speed_xyz.z = speed_xyz_last.z + rot_acc.z * SAMPLE_TIME;
    }
    else //静止状态速度为0
    {
        speed_xyz.x = 0;
        speed_xyz.y = 0;
        speed_xyz.z = 0;
    }
//    printf("%f,%f,%f\r\n", speed_xyz.x, speed_xyz.y, speed_xyz.z);
	
	
	
	
	
	/*********************************漂移之后的数值不对劲************************/
    //对于非静止状态下的速度，计算漂移,把它从速度中移除
    if (stationary == 0) 
    {
        count_time++;

        speed_xyz_drift.x += speed_xyz.x;
        speed_xyz_drift.y += speed_xyz.y;
        speed_xyz_drift.z += speed_xyz.z;
        if(count_time == 10)   //每50ms更新一次 速度漂移率 = 速度 / 时间
        {
            speed_xyz_drift.x = speed_xyz.x / 50;
            speed_xyz_drift.y = speed_xyz.y / 50;
            speed_xyz_drift.z = speed_xyz.z / 50;

            speed_xyz_drift_last = speed_xyz_drift; 
            speed_xyz_drift = (Vector3){0, 0, 0};
            count_time = 0;
        }
    }

//    //去除速度漂移
//    speed_xyz.x -= speed_xyz_drift_last.x * SAMPLE_TIME;
//    speed_xyz.y -= speed_xyz_drift_last.y * SAMPLE_TIME;
//    speed_xyz.z -= speed_xyz_drift_last.z * SAMPLE_TIME;
	
//    printf("%f,%f,%f\r\n", speed_xyz.x, speed_xyz.y, speed_xyz.z);
	
	
	
    
//位置计算/**************************************为什么会自己清零？？？？**************************/
    position_xyz.x += fabs(speed_xyz.x * SAMPLE_TIME);
    position_xyz.y += fabs(speed_xyz.y * SAMPLE_TIME);
    position_xyz.z += fabs(speed_xyz.z * SAMPLE_TIME);
	printf("%f,%f,%f\r\n", position_xyz.x, position_xyz.y, position_xyz.z);
    //更新速度
    speed_xyz_last = speed_xyz;

    return position_xyz;    
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


// 使用四元数旋转向量
Vector3 rotate_vector_by_quaternion(Vector3 v3, Quaternion quart) 
{
    // 将向量转换为四元数，w = 0
    Quaternion q_v = {0, v3.x, v3.y, v3.z};     
    // 共轭四元数
    Quaternion q_conj = quat_conj(quart);
    // 矩阵相乘 v′ = q⋅v⋅q* 
    Quaternion q_result = quat_multiply(quat_multiply(quart, q_v), q_conj);

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


