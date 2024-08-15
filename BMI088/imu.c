#include "imu.h"
#include <math.h>

#define halfT 0.0085f

float Kp = 1.5f;
float Ki = 0.0008f;


volatile float gyro_x;
volatile float gyro_y;
volatile float gyro_z;
double gyro_z1;
float accel_x;
float accel_y;
float accel_z;

float Pitch;
float Roll;
float Yaw;

typedef struct 
{  
    float _cutoff_freq;         // 截止频率 (Hz)  
    float _a11;                 // 反馈系数 a1  
    float _a21;                 // 反馈系数 a2  
    float _b01;                 // 前馈系数 b0  
    float _b11;                 // 前馈系数 b1  
    float _b21;                 // 前馈系数 b2  
    float _delay_element_11;    // 延迟元素1  
    float _delay_element_21;    // 延迟元素2  
} LPF;


Quaternion quart = {1.0f, 0.0f, 0.0f, 0.0f};

float  exInt = 0, eyInt = 0, ezInt = 0; //叉积计算误差的累计积分

void AHRS(float gx,float gy,float gz,float ax,float ay,float az)
{		
	float recipNorm;
	static float exInt=0;
	static float eyInt=0;
	static float ezInt=0;
	float vx;
	float vy;
	float vz;
	float ex;
	float ey;
	float ez;

	recipNorm = invSqrt(ax*ax+ay*ay+az*az);
	ax = ax * recipNorm;
	ay = ay * recipNorm;
	az = az * recipNorm;

	vx = 2*(quart.q1*quart.q3-quart.q0*quart.q2);
	vy = 2*(quart.q0*quart.q1+quart.q2*quart.q3);
	vz = quart.q0*quart.q0 - quart.q1*quart.q1- quart.q2*quart.q2 + quart.q3*quart.q3;

	ex = (ay*vz - az*vy);
	ey = (az*vx - ax*vz);
	ez = (ax*vy - ay*vx);

	exInt = exInt + ex * Ki;
	eyInt = eyInt + ey * Ki;
	ezInt = ezInt + ez * Ki;

	gx = gx + Kp*ex + exInt;
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;

	quart.q0 = quart.q0 +(-quart.q1*gx-quart.q2*gy-quart.q3*gz)*halfT;
	quart.q1 = quart.q1 +(quart.q0*gx+quart.q2*gz-quart.q3*gy)*halfT;
	quart.q2 = quart.q2 +(quart.q0*gy-quart.q1*gz+quart.q3*gx)*halfT;
	quart.q3 = quart.q3 +(quart.q0*gz+quart.q1*gy-quart.q2*gx)*halfT;

	recipNorm = invSqrt(quart.q0*quart.q0 + quart.q1*quart.q1 + quart.q2*quart.q2 + quart.q3*quart.q3);
	quart.q0 = quart.q0 * recipNorm;
	quart.q1 = quart.q1 * recipNorm;
	quart.q2 = quart.q2 * recipNorm;
	quart.q3 = quart.q3 * recipNorm;

//	#ifdef POSITION_CALC	
		Pitch = asinf(2 * quart.q1*quart.q3 - 2 * quart.q0*quart.q2) * 57.3f;
		Roll  = atan2f(2 * quart.q2*quart.q3 + 2 * quart.q0*quart.q1, -2 * quart.q1*quart.q1 - 2 * quart.q2*quart.q2 + 1) * 57.3f;
		//Yaw   = -atan2f(2 * q1*q2 + 2 * q0*q3, -2 * q2*q2 - 2 * q3*q3 + 1) * 57.3f;
//	#endif
		
}



LPF My_Filter;

/**
  * @brief  二阶滤波函数参数设置       
  * @param[in]   原始采样数据        
  * @param[in]   样本数据频率       
  * @param[in]   截止频率
  * @retval         none
	* @note
  */
void LPF2_ParamSet(double sample_freq, double cutoff_freq)
{               
    double fr = 0;
    double ohm = 0;
    double c = 0;

    fr = sample_freq / cutoff_freq;
    ohm = tanf(M_PI_F / fr);
    c = 1.0 + 2.0 * cosf(M_PI_F / 4.0) * ohm + ohm * ohm;

	/* 直接给出截止频率 */
    My_Filter._cutoff_freq = cutoff_freq;
    if (My_Filter._cutoff_freq > 0.0f)
    {
        My_Filter._b01 = ohm * ohm / c;
        My_Filter._b11 = 2.0f * My_Filter._b01;
        My_Filter._b21 = My_Filter._b01;
        My_Filter._a11 = 2.0f * (ohm * ohm - 1.0f) / c;
        My_Filter._a21 = (1.0 - 2.0 * cosf(M_PI_F / 4.0) * ohm + ohm * ohm) / c;
    }
}

/**
  * @brief  二阶低通滤波函数   
  * @param[in]   对应传感器   
  * @param[in]   样本数据
  * @retval 二阶低通滤波函数
  */
float LPF2_Calculate(float sample)
{
    float delay_element_0 = 0, output = 0;
    if (My_Filter._cutoff_freq <= 0.0f)
    {
        // no filtering
        return sample;
    }
    else
    {
        delay_element_0 = sample - My_Filter._delay_element_11 * My_Filter._a11 - My_Filter._delay_element_21 * My_Filter._a21;
        if (isnan(delay_element_0) || isinf(delay_element_0))
        {
            //不允许结果是nan或者无穷大
            delay_element_0 = sample;
        }

        output = delay_element_0 * My_Filter._b01 + My_Filter._delay_element_11 * My_Filter._b11 + My_Filter._delay_element_21 * My_Filter._b21;
        
        //更新延迟元素
        My_Filter._delay_element_21 = My_Filter._delay_element_11;
        My_Filter._delay_element_11 = delay_element_0;
       
        return output;
    }
}   


void ACC_XYZ_Window_Filter(struct bmi08x_sensor_data *ACC_xyz)  //对加速度数据进行滑动窗口滤波
{  
    static int index = 0; // 用于记录最近写入的位置  
    static float filter_buf_x[FILTER_N + 1]; // X轴滤波缓冲区  
    static float filter_buf_y[FILTER_N + 1]; // Y轴滤波缓冲区  
    static float filter_buf_z[FILTER_N + 1]; // Z轴滤波缓冲区  
    // 将新数据写入到缓冲区并更新  
    filter_buf_x[index] = ACC_xyz->x;  
    filter_buf_y[index] = ACC_xyz->y;  
    filter_buf_z[index] = ACC_xyz->z;  

    // 更新索引，循环使用  
    index = (index + 1) % FILTER_N;  

    // 计算每个方向的滤波值  
    float sum_x = 0, sum_y = 0, sum_z = 0;  
    for (int i = 0; i < FILTER_N; i++)  
    {  
        sum_x += filter_buf_x[i];  
        sum_y += filter_buf_y[i];  
        sum_z += filter_buf_z[i];  
    }  

    // 得到滤波后的值  
    float filtered_x = sum_x / FILTER_N;  
    float filtered_y = sum_y / FILTER_N;  
    float filtered_z = sum_z / FILTER_N;  

    // 更新原始结构体数据  
    ACC_xyz->x = filtered_x;   
    ACC_xyz->y = filtered_y;  
    ACC_xyz->z = filtered_z;   
} 

////对陀螺仪数据进行滑动窗口滤波
//void GYRO_XYZ_Window_Filter(struct bmi08x_sensor_data *GYRO_xyz)  
//{  
//    static int index = 0; // 用于记录最近写入的位置  
//    static float filter_buf_x[FILTER_N + 1]; // X轴滤波缓冲区  
//    static float filter_buf_y[FILTER_N + 1]; // Y轴滤波缓冲区  
//    static float filter_buf_z[FILTER_N + 1]; // Z轴滤波缓冲区  
//    // 将新数据写入到缓冲区并更新  
//    filter_buf_x[index] = GYRO_xyz->x;  
//    filter_buf_y[index] = GYRO_xyz->y;  
//    filter_buf_z[index] = GYRO_xyz->z;  

//    // 更新索引，循环使用  
//    index = (index + 1) % FILTER_N;  

//    // 计算每个方向的滤波值  
//    float sum_x = 0, sum_y = 0, sum_z = 0;  
//    for (int i = 0; i < FILTER_N; i++)  
//    {  
//        sum_x += filter_buf_x[i];  
//        sum_y += filter_buf_y[i];  
//        sum_z += filter_buf_z[i];  
//    }  

//    // 得到滤波后的值  
//    float filtered_x = sum_x / FILTER_N;  
//    float filtered_y = sum_y / FILTER_N;  
//    float filtered_z = sum_z / FILTER_N;  

//    // 更新原始结构体数据  
//    GYRO_xyz->x = filtered_x;   
//    GYRO_xyz->y = filtered_y;  
//    GYRO_xyz->z = filtered_z;   
//}


float Single_Window_Filter(float Sample) //窗口滤波函数
{
    static float filter_buf[FILTER_N_SINGLE + 1];
    int i;    
    float filter_sum = 0;
    filter_buf[FILTER_N_SINGLE] = Sample;
    for(i = 0; i < FILTER_N_SINGLE; i++) 
    {
        filter_buf[i] = filter_buf[i + 1]; // 所有数据左移，低位仍掉
        filter_sum += filter_buf[i];
    }
    return (float)(filter_sum / FILTER_N_SINGLE);
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


	
	
	
	
	
	
	
	
	
	
	
	
