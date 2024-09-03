#ifndef __VL53L0_I2C_H
#define __VL53L0_I2C_H

#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;
//ALIENTEK ������STM32F429������
//VL53L0X IIC���� ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2017/7/1
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) �������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////

//IO��������
#define VL_SDA_IN()  {GPIOB->MODER&=~(3<<(10*2));GPIOB->MODER|=0<<10*2;}	//PB10����ģʽ
#define VL_SDA_OUT() {GPIOB->MODER&=~(3<<(10*2));GPIOB->MODER|=1<<10*2;}    //PB10���ģʽ


//IO��������	 
#define VL_IIC_SCL    PBout(11) 		//SCL
#define VL_IIC_SDA    PBout(10) 		//SDA	 
#define VL_READ_SDA   PBin(10) 		    //����SDA 

//״̬
#define STATUS_OK       0x00
#define STATUS_FAIL     0x01

//IIC��������
void VL53L0X_i2c_init(void);//��ʼ��IIC��IO��

u8 VL53L0X_write_byte(u8 address,u8 index,u8 data);              //IICдһ��8λ����
u8 VL53L0X_write_word(u8 address,u8 index,u16 data);             //IICдһ��16λ����
u8 VL53L0X_write_dword(u8 address,u8 index,u32 data);            //IICдһ��32λ����
u8 VL53L0X_write_multi(u8 address, u8 index,u8 *pdata,u16 count);//IIC����д

u8 VL53L0X_read_byte(u8 address,u8 index,u8 *pdata);             //IIC��һ��8λ����
u8 VL53L0X_read_word(u8 address,u8 index,u16 *pdata);            //IIC��һ��16λ����
u8 VL53L0X_read_dword(u8 address,u8 index,u32 *pdata);           //IIC��һ��32λ����
u8 VL53L0X_read_multi(u8 address,u8 index,u8 *pdata,u16 count);  //IIC������


#endif 

