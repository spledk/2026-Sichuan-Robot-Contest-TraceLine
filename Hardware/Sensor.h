#ifndef Sensor_H
#define Sensor_H
#include "stm32f10x.h"

enum Location_Status{
	on=1,
	off=0
};

enum Sensor_List{
	L2=0,
	L1,
	R1,
	R2
};

typedef struct{
	enum Location_Status L2;
	enum Location_Status L1;
	enum Location_Status R1;
	enum Location_Status R2;
}status;

typedef struct{
	float error;
	uint8_t OnLineNum;
}ErrorInformation;

void Sensor_Init(void);
ErrorInformation Sensor_GetErrorInformation(void);
void Sensor_Blind(enum Sensor_List sensor);
void Sensor_Reset(void);

#endif
