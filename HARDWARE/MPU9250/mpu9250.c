#include "mpu9250.h"
#include "myiic.h"
#include "delay.h"
#include <stdio.h>




//得到温度值
//返回值:温度值(扩大了100倍)
short MPU_Get_Temperature(void)
{
    u8 buf[2]; 
    short raw;
	float temp;
	MPU_Read_Len(MPU9250_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((u16)buf[0]<<8)|buf[1];  
    temp=21+((double)raw)/333.87;  
    return temp*100;;
}

//得到陀螺仪值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    u8 buf[6],res; 
	res=MPU_Read_Len(MPU9250_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=((u16)buf[0]<<8)|buf[1];  
		*gy=((u16)buf[2]<<8)|buf[3];  
		*gz=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}

//得到加速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    u8 buf[6],res;  
	res=MPU_Read_Len(MPU9250_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((u16)buf[0]<<8)|buf[1];  
		*ay=((u16)buf[2]<<8)|buf[3];  
		*az=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}

//得到磁力计值(原始值)
//mx,my,mz:磁力计x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
u8 MPU_Get_Magnetometer(short *mx,short *my,short *mz)
{
    u8 buf[6],res;  
	res=MPU_Read_Len(AK8963_ADDR,MAG_XOUT_L,6,buf);
	if(res==0)
	{
		*mx=((u16)buf[1]<<8)|buf[0];  
		*my=((u16)buf[3]<<8)|buf[2];  
		*mz=((u16)buf[5]<<8)|buf[4];
	} 	
    MPU_Write_Byte(AK8963_ADDR,MAG_CNTL1,0X11); //AK8963每次读完以后都需要重新设置为单次测量模式
    return res;;
}

//IIC连续写
//addr:器件地址 
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
    u8 i;
    SoftSim_IIC_Start();
    SoftSim_IIC_Send_Byte((addr<<1)|0); //发送器件地址+写命令
    if(SoftSim_IIC_Wait_Ack())          //等待应答
    {
        SoftSim_IIC_Stop();
        return 1;
    }
    SoftSim_IIC_Send_Byte(reg);         //写寄存器地址
    SoftSim_IIC_Wait_Ack();             //等待应答
    for(i=0;i<len;i++)
    {
        SoftSim_IIC_Send_Byte(buf[i]);  //发送数据
        if(SoftSim_IIC_Wait_Ack())      //等待ACK
        {
            SoftSim_IIC_Stop();
            return 1;
        }
    }
    SoftSim_IIC_Stop();
    return 0;
} 

//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
    SoftSim_IIC_Start();
    SoftSim_IIC_Send_Byte((addr<<1)|0); //发送器件地址+写命令
    if(SoftSim_IIC_Wait_Ack())          //等待应答
    {
        SoftSim_IIC_Stop();
        return 1;
    }
    SoftSim_IIC_Send_Byte(reg);         //写寄存器地址
    SoftSim_IIC_Wait_Ack();             //等待应答
	SoftSim_IIC_Start();                
    SoftSim_IIC_Send_Byte((addr<<1)|1); //发送器件地址+读命令
    SoftSim_IIC_Wait_Ack();             //等待应答
    while(len)
    {
        if(len==1)*buf=SoftSim_IIC_Read_Byte(0);//读数据,发送nACK 
		else *buf=SoftSim_IIC_Read_Byte(1);		//读数据,发送ACK  
		len--;
		buf++;  
    }
    SoftSim_IIC_Stop();                 //产生一个停止条件
    return 0;       
}

//IIC写一个字节 
//devaddr:器件IIC地址
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
u8 MPU_Write_Byte(u8 addr,u8 reg,u8 data)
{
    SoftSim_IIC_Start();
    SoftSim_IIC_Send_Byte((addr<<1)|0); //发送器件地址+写命令
    if(SoftSim_IIC_Wait_Ack())          //等待应答
    {
        SoftSim_IIC_Stop();
        return 1;
    }
    SoftSim_IIC_Send_Byte(reg);         //写寄存器地址
    SoftSim_IIC_Wait_Ack();             //等待应答
    SoftSim_IIC_Send_Byte(data);        //发送数据
    if(SoftSim_IIC_Wait_Ack())          //等待ACK
    {
        SoftSim_IIC_Stop();
        return 1;
    }
    SoftSim_IIC_Stop();
    return 0;
}

