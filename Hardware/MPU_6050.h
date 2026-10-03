#ifndef MPU_6050_H
#define	MPU_6050_H
#include "stm32f10x.h"

//GYRO_CONFIG=0x00(±250°/s) + CONFIG=0x06 + SMPLRT_DIV=9  =>  100Hz采样率
#define MPU_6050_GYRO_SENS  131.0f			//LSB/(°/s)
#define MPU_6050_SAMPLE_DT  (1.0f/100.0f)	//一个样本代表10ms

enum Data_Status{
	Old=0,
	New
};

struct MPU_6050_Data{
	int16_t AccX;
	int16_t AccY;
	int16_t AccZ;
	int16_t GyroX;
	int16_t GyroY;
	int16_t GyroZ;
};

struct MPU_6050_TurnNeedData{
	enum Data_Status IsNew;
	int16_t GyroZ;
};

void MPU_6050_WriteReg(uint8_t RegAddress,uint8_t Data);
uint8_t MPU_6050_ReadReg(uint8_t RegAddress);
void MPU_6050_Init(void);
void MPU_6050_GyroErrorCorrect(void);
struct MPU_6050_Data MPU_6050_GetData(void);
struct MPU_6050_TurnNeedData MPU_6050_GetTurnNeedData(void);
void MPU_6050_YawAngle_Update(const struct MPU_6050_TurnNeedData *Data);
float MPU_6050_GetYaw(void);
void MPU_6050_SetYaw(float targetdegree);

#endif
