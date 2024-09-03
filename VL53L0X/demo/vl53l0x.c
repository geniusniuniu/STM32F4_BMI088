#include "vl53l0x.h"
#include "GPIO.h"
#include "vl53l0x_gen.h"

VL53L0X_Dev_t vl53l0x_dev[3]; //设备I2C数据参数


VL53L0X_DeviceInfo_t vl53l0x_dev_info;//设备ID版本信息
uint8_t AjustOK=0;//校准标志位

//VL53L0X各测量模式参数
//0：默认;1:高精度;2:长距离;3:高速
mode_data Mode_data[]=
{
    {(FixPoint1616_t)(0.25*65536), 
	 (FixPoint1616_t)(18*65536),
	 33000,
	 14,
	 10},//默认
		
	{(FixPoint1616_t)(0.25*65536) ,
	 (FixPoint1616_t)(18*65536),
	 200000, 
	 14,
	 10},//高精度
		
    {(FixPoint1616_t)(0.1*65536) ,
	 (FixPoint1616_t)(60*65536),
	 33000,
	 18,
	 14},//长距离
	
    {(FixPoint1616_t)(0.25*65536) ,
	 (FixPoint1616_t)(32*65536),
	 20000,
	 14,
	 10},//高速
		
};

//API错误信息打印
//Status：详情看VL53L0X_Error参数的定义
void print_pal_error(VL53L0X_Error Status)
{
	
	char buf[VL53L0X_MAX_STRING_LENGTH];
	
	VL53L0X_GetPalErrorString(Status,buf);//根据Status状态获取错误信息字符串
	
    printf("API Status: %i : %s\r\n",Status, buf);//打印状态和错误信息
	
}


//配置VL53L0X设备I2C地址
//dev:设备I2C参数结构体
//newaddr:设备新I2C地址
VL53L0X_Error vl53l0x_Addr_set(VL53L0X_Dev_t *dev,uint8_t newaddr)
{
	uint16_t Id;
	uint8_t FinalAddress;
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	u8 sta=0x00;
	
	FinalAddress = newaddr;
	
	if(FinalAddress==dev->I2cDevAddr)//新设备I2C地址与旧地址一致,直接退出
	{
		return VL53L0X_ERROR_NONE;
	}
	//在进行第一个寄存器访问之前设置I2C标准模式(400Khz)
	Status = VL53L0X_WrByte(dev,0x88,0x00);
	if(Status!=VL53L0X_ERROR_NONE) 
	{
		sta=0x01;//设置I2C标准模式出错
		goto set_error;
	}
	//尝试使用默认的0x52地址读取一个寄存器
	Status = VL53L0X_RdWord(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
	if(Status!=VL53L0X_ERROR_NONE) 
	{
		sta=0x02;//读取寄存器出错
		goto set_error;
	}
	if(Id == 0xEEAA)
	{
		//设置设备新的I2C地址
		Status = VL53L0X_SetDeviceAddress(dev,newaddr);
		if(Status!=VL53L0X_ERROR_NONE) 
		{
			sta=0x03;//设置I2C地址出错
			goto set_error;
		}
		//修改参数结构体的I2C地址
		dev->I2cDevAddr = newaddr;
		//检查新的I2C地址读写是否正常
		Status = VL53L0X_RdWord(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
		if(Status!=VL53L0X_ERROR_NONE) 
		{
			sta=0x04;//新I2C地址读写出错
			goto set_error;
		}	
	}
	set_error:
	if(Status!=VL53L0X_ERROR_NONE)
	{
		print_pal_error(Status);//打印错误信息
	}
	if(sta!=0)
	  printf("sta:0x%x\r\n",sta);
	return Status;
}

//vl53l0x复位函数
//dev:设备I2C参数结构体
void vl53l0x_reset(VL53L0X_Dev_t *dev,char Xshut_Pin)
{
	uint8_t addr;
	addr = dev->I2cDevAddr;//保存设备原I2C地址
    PAout(Xshut_Pin)=0;//失能VL53L0X
	HAL_Delay(30);
	PAout(Xshut_Pin)=1;//使能VL53L0X,让传感器处于工作(I2C地址会恢复默认0X52)
	HAL_Delay(30);	
	dev->I2cDevAddr=0x52;
	vl53l0x_Addr_set(dev,addr);//设置VL53L0X传感器原来上电前原I2C地址
	VL53L0X_DataInit(dev);	
}

void XShut_PinInit(void)
{
	GPIO_InitTypeDef GPIO_Initure;
	
	GPIO_Initure.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5; //PA3，A4，A5
    GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;  			 //推挽输出
    GPIO_Initure.Pull = GPIO_PULLUP;          			 //上拉
    GPIO_Initure.Speed = GPIO_SPEED_HIGH;     			 //高速
	
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);
	
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);  //初始化，Disable
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);	 
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);	
}

//初始化vl53l0x
//dev:设备I2C参数结构体
VL53L0X_Error vl53l0x_init(VL53L0X_Dev_t *dev,char Xshut_Pin)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	VL53L0X_Dev_t *pMyDevice = dev;
	
	pMyDevice->I2cDevAddr = VL53L0X_Addr;	//I2C地址(上电默认0x52)
	pMyDevice->comms_type = 1;           	//I2C通信模式
	pMyDevice->comms_speed_khz = 400;    	//I2C通信速率
	
	PAout(Xshut_Pin)=0;//失能VL53L0X
	HAL_Delay(10);
	PAout(Xshut_Pin)=1;//使能VL53L0X,让传感器处于工作
	HAL_Delay(10);
	
    vl53l0x_Addr_set(pMyDevice,0x54);//设置VL53L0X传感器I2C地址
    if(Status!=VL53L0X_ERROR_NONE) goto error;
	Status = VL53L0X_DataInit(pMyDevice);//设备初始化
	if(Status!=VL53L0X_ERROR_NONE) goto error;
	HAL_Delay(2);
	Status = VL53L0X_GetDeviceInfo(pMyDevice,&vl53l0x_dev_info);//获取设备ID信息
    if(Status!=VL53L0X_ERROR_NONE) goto error;
	
	if(Vl53l0x_data.adjustok==0xAA)//已校准 
	  AjustOK=1;	
	else //没校准	
	  AjustOK=0;
	
	error:
	if(Status!=VL53L0X_ERROR_NONE)
	{
		print_pal_error(Status);//打印错误信息
		return Status;
	}
  	
	return Status;
}


//VL53L0X主测试程序
//void vl53l0x_test(void)
//{   
//	 u8 i=0;
//	 u8 key=0;
//	 while(vl53l0x_init(&vl53l0x_dev))//vl53l0x初始化
//	 {
//		printf("VL53L0X Error!!!\n\r");
//		LCD_ShowString(30,140,200,16,16,"VL53L0X Error!!!");
//	    HAL_Delay(500);
//		LCD_ShowString(30,140,200,16,16,"                ");
//		HAL_Delay(500);
//		LED0=!LED0;//DS0闪烁
//	 }
//	 printf("VL53L0X OK\r\n");
//	 LCD_ShowString(30,140,200,16,16,"VL53L0X OK");
//	 vl53l0x_mtest_ui();//主菜单显示
//	 while(1)
//	 {
//		 
//		 key = KEY_Scan(0);
//		 if(key)
//		 {
//			  switch(key)
//			  {
//				  case WKUP_PRES:  vl53l0x_calibration_test(&vl53l0x_dev);           break;//校准模式
//				  case KEY1_PRES:  vl53l0x_general_test(&vl53l0x_dev);               break;//普通测量模式
//				  case KEY0_PRES:  vl53l0x_interrupt_test(&vl53l0x_dev);             break;//中断测量模式  
//			  }
//			  vl53l0x_mtest_ui();
//		 }
//		 i++;
//		 if(i==5)
//		 {
//			 i=0;
//			 LED0=!LED0;
//		 }
//		 HAL_Delay(50);
//		 
//	 }
//}



//获取一次测量距离数据
void One_measurement()
{
	VL53L0X_PerformSingleRangingMeasurement(&vl53l0x_dev[0],&vl53l0x_data);	
	printf("%4d\r\n",vl53l0x_data.RangeMilliMeter);
		
}


//----------以下函数为USMART调用------------//

//获取vl53l0x传感器ID信息
void vl53l0x_info(void)
{
//	printf("\r\n-------vl53l0x传感器设备信息-------\r\n\r\n");
//	printf("  Name:%s\r\n",vl53l0x_dev_info.Name);
//	printf("  Addr:0x%x\r\n",Xaxis_vl53l0x_dev.I2cDevAddr);
//	printf("  ProductId:%s\r\n",vl53l0x_dev_info.ProductId);
//	printf("  RevisionMajor:0x%x\r\n",vl53l0x_dev_info.ProductRevisionMajor);
//	printf("  RevisionMinor:0x%x\r\n",vl53l0x_dev_info.ProductRevisionMinor);
//	printf("\r\n-----------------------------------\r\n");
}


