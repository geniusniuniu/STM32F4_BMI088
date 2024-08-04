#ifndef __IMU_H
#define	__IMU_H
#include "main.h"


#define squa( Sq )        (((float)Sq)*((float)Sq))


typedef struct{
	int16_t accX;
	int16_t accY;
	int16_t accZ;
	int16_t gyroX;
	int16_t gyroY;
	int16_t gyroZ;
}_st_Mpu;

typedef struct{
	float roll;
	float pitch;
	float yaw;
}_st_AngE;




void GetAngle(const _st_Mpu *pMpu,_st_AngE *pAngE, float dt);
void imu_rest(void);

#endif
