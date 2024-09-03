#include "vl53l0x.h"
#include "GPIO.h"
#include "vl53l0x_gen.h"

VL53L0X_Dev_t vl53l0x_dev[3]; //�豸I2C���ݲ���


VL53L0X_DeviceInfo_t vl53l0x_dev_info;//�豸ID�汾��Ϣ
uint8_t AjustOK=0;//У׼��־λ

//VL53L0X������ģʽ����
//0��Ĭ��;1:�߾���;2:������;3:����
mode_data Mode_data[]=
{
    {(FixPoint1616_t)(0.25*65536), 
	 (FixPoint1616_t)(18*65536),
	 33000,
	 14,
	 10},//Ĭ��
		
	{(FixPoint1616_t)(0.25*65536) ,
	 (FixPoint1616_t)(18*65536),
	 200000, 
	 14,
	 10},//�߾���
		
    {(FixPoint1616_t)(0.1*65536) ,
	 (FixPoint1616_t)(60*65536),
	 33000,
	 18,
	 14},//������
	
    {(FixPoint1616_t)(0.25*65536) ,
	 (FixPoint1616_t)(32*65536),
	 20000,
	 14,
	 10},//����
		
};

//API������Ϣ��ӡ
//Status�����鿴VL53L0X_Error�����Ķ���
void print_pal_error(VL53L0X_Error Status)
{
	
	char buf[VL53L0X_MAX_STRING_LENGTH];
	
	VL53L0X_GetPalErrorString(Status,buf);//����Status״̬��ȡ������Ϣ�ַ���
	
    printf("API Status: %i : %s\r\n",Status, buf);//��ӡ״̬�ʹ�����Ϣ
	
}


//����VL53L0X�豸I2C��ַ
//dev:�豸I2C�����ṹ��
//newaddr:�豸��I2C��ַ
VL53L0X_Error vl53l0x_Addr_set(VL53L0X_Dev_t *dev,uint8_t newaddr)
{
	uint16_t Id;
	uint8_t FinalAddress;
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	u8 sta=0x00;
	
	FinalAddress = newaddr;
	
	if(FinalAddress==dev->I2cDevAddr)//���豸I2C��ַ��ɵ�ַһ��,ֱ���˳�
	{
		return VL53L0X_ERROR_NONE;
	}
	//�ڽ��е�һ���Ĵ�������֮ǰ����I2C��׼ģʽ(400Khz)
	Status = VL53L0X_WrByte(dev,0x88,0x00);
	if(Status!=VL53L0X_ERROR_NONE) 
	{
		sta=0x01;//����I2C��׼ģʽ����
		goto set_error;
	}
	//����ʹ��Ĭ�ϵ�0x52��ַ��ȡһ���Ĵ���
	Status = VL53L0X_RdWord(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
	if(Status!=VL53L0X_ERROR_NONE) 
	{
		sta=0x02;//��ȡ�Ĵ�������
		goto set_error;
	}
	if(Id == 0xEEAA)
	{
		//�����豸�µ�I2C��ַ
		Status = VL53L0X_SetDeviceAddress(dev,newaddr);
		if(Status!=VL53L0X_ERROR_NONE) 
		{
			sta=0x03;//����I2C��ַ����
			goto set_error;
		}
		//�޸Ĳ����ṹ���I2C��ַ
		dev->I2cDevAddr = newaddr;
		//����µ�I2C��ַ��д�Ƿ�����
		Status = VL53L0X_RdWord(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
		if(Status!=VL53L0X_ERROR_NONE) 
		{
			sta=0x04;//��I2C��ַ��д����
			goto set_error;
		}	
	}
	set_error:
	if(Status!=VL53L0X_ERROR_NONE)
	{
		print_pal_error(Status);//��ӡ������Ϣ
	}
	if(sta!=0)
	  printf("sta:0x%x\r\n",sta);
	return Status;
}

//vl53l0x��λ����
//dev:�豸I2C�����ṹ��
void vl53l0x_reset(VL53L0X_Dev_t *dev,char Xshut_Pin)
{
	uint8_t addr;
	addr = dev->I2cDevAddr;//�����豸ԭI2C��ַ
    PAout(Xshut_Pin)=0;//ʧ��VL53L0X
	HAL_Delay(30);
	PAout(Xshut_Pin)=1;//ʹ��VL53L0X,�ô��������ڹ���(I2C��ַ��ָ�Ĭ��0X52)
	HAL_Delay(30);	
	dev->I2cDevAddr=0x52;
	vl53l0x_Addr_set(dev,addr);//����VL53L0X������ԭ���ϵ�ǰԭI2C��ַ
	VL53L0X_DataInit(dev);	
}

void XShut_PinInit(void)
{
	GPIO_InitTypeDef GPIO_Initure;
	
	GPIO_Initure.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5; //PA3��A4��A5
    GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;  			 //�������
    GPIO_Initure.Pull = GPIO_PULLUP;          			 //����
    GPIO_Initure.Speed = GPIO_SPEED_HIGH;     			 //����
	
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);
	
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);  //��ʼ����Disable
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);	 
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);	
}

//��ʼ��vl53l0x
//dev:�豸I2C�����ṹ��
VL53L0X_Error vl53l0x_init(VL53L0X_Dev_t *dev,char Xshut_Pin)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	VL53L0X_Dev_t *pMyDevice = dev;
	
	pMyDevice->I2cDevAddr = VL53L0X_Addr;	//I2C��ַ(�ϵ�Ĭ��0x52)
	pMyDevice->comms_type = 1;           	//I2Cͨ��ģʽ
	pMyDevice->comms_speed_khz = 400;    	//I2Cͨ������
	
	PAout(Xshut_Pin)=0;//ʧ��VL53L0X
	HAL_Delay(10);
	PAout(Xshut_Pin)=1;//ʹ��VL53L0X,�ô��������ڹ���
	HAL_Delay(10);
	
    vl53l0x_Addr_set(pMyDevice,0x54);//����VL53L0X������I2C��ַ
    if(Status!=VL53L0X_ERROR_NONE) goto error;
	Status = VL53L0X_DataInit(pMyDevice);//�豸��ʼ��
	if(Status!=VL53L0X_ERROR_NONE) goto error;
	HAL_Delay(2);
	Status = VL53L0X_GetDeviceInfo(pMyDevice,&vl53l0x_dev_info);//��ȡ�豸ID��Ϣ
    if(Status!=VL53L0X_ERROR_NONE) goto error;
	
	if(Vl53l0x_data.adjustok==0xAA)//��У׼ 
	  AjustOK=1;	
	else //ûУ׼	
	  AjustOK=0;
	
	error:
	if(Status!=VL53L0X_ERROR_NONE)
	{
		print_pal_error(Status);//��ӡ������Ϣ
		return Status;
	}
  	
	return Status;
}


//VL53L0X�����Գ���
//void vl53l0x_test(void)
//{   
//	 u8 i=0;
//	 u8 key=0;
//	 while(vl53l0x_init(&vl53l0x_dev))//vl53l0x��ʼ��
//	 {
//		printf("VL53L0X Error!!!\n\r");
//		LCD_ShowString(30,140,200,16,16,"VL53L0X Error!!!");
//	    HAL_Delay(500);
//		LCD_ShowString(30,140,200,16,16,"                ");
//		HAL_Delay(500);
//		LED0=!LED0;//DS0��˸
//	 }
//	 printf("VL53L0X OK\r\n");
//	 LCD_ShowString(30,140,200,16,16,"VL53L0X OK");
//	 vl53l0x_mtest_ui();//���˵���ʾ
//	 while(1)
//	 {
//		 
//		 key = KEY_Scan(0);
//		 if(key)
//		 {
//			  switch(key)
//			  {
//				  case WKUP_PRES:  vl53l0x_calibration_test(&vl53l0x_dev);           break;//У׼ģʽ
//				  case KEY1_PRES:  vl53l0x_general_test(&vl53l0x_dev);               break;//��ͨ����ģʽ
//				  case KEY0_PRES:  vl53l0x_interrupt_test(&vl53l0x_dev);             break;//�жϲ���ģʽ  
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



//��ȡһ�β�����������
void One_measurement()
{
	VL53L0X_PerformSingleRangingMeasurement(&vl53l0x_dev[0],&vl53l0x_data);	
	printf("%4d\r\n",vl53l0x_data.RangeMilliMeter);
		
}


//----------���º���ΪUSMART����------------//

//��ȡvl53l0x������ID��Ϣ
void vl53l0x_info(void)
{
//	printf("\r\n-------vl53l0x�������豸��Ϣ-------\r\n\r\n");
//	printf("  Name:%s\r\n",vl53l0x_dev_info.Name);
//	printf("  Addr:0x%x\r\n",Xaxis_vl53l0x_dev.I2cDevAddr);
//	printf("  ProductId:%s\r\n",vl53l0x_dev_info.ProductId);
//	printf("  RevisionMajor:0x%x\r\n",vl53l0x_dev_info.ProductRevisionMajor);
//	printf("  RevisionMinor:0x%x\r\n",vl53l0x_dev_info.ProductRevisionMinor);
//	printf("\r\n-----------------------------------\r\n");
}

