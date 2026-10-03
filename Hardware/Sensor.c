#include "stm32f10x.h"
#include "Sensor.h"

#define Sensor_GPIO_Port GPIOA
#define Sensor_GPIO_CLK  RCC_APB2Periph_GPIOA

//传感器接口非顺序排列
#define Sensor_L2_Pin GPIO_Pin_4		
#define Sensor_L1_Pin GPIO_Pin_5		
#define Sensor_R1_Pin GPIO_Pin_3		
#define Sensor_R2_Pin GPIO_Pin_2	

//现实测量值 单位cm
const float Weight[4]={-3.15,-0.35,0.35,3.15};
float weight[4]={-3.15,-0.35,0.35,3.15};

void Sensor_Init(void){
	RCC_APB2PeriphClockCmd(Sensor_GPIO_CLK, ENABLE);
	
	GPIO_InitTypeDef GPIOInitStruct={
		.GPIO_Pin=Sensor_L2_Pin | Sensor_L1_Pin | Sensor_R1_Pin | Sensor_R2_Pin,
		.GPIO_Mode=GPIO_Mode_IPD,
		.GPIO_Speed=GPIO_Speed_50MHz
	};
	GPIO_Init(Sensor_GPIO_Port,&GPIOInitStruct);		
}
//根据实际传值判断是否取反
status Sensor_Location_Status(void){
	status Sensed_Location_Status={
		.L2=(enum Location_Status)!GPIO_ReadInputDataBit(Sensor_GPIO_Port,Sensor_L2_Pin),
		.L1=(enum Location_Status)!GPIO_ReadInputDataBit(Sensor_GPIO_Port,Sensor_L1_Pin),
		.R1=(enum Location_Status)!GPIO_ReadInputDataBit(Sensor_GPIO_Port,Sensor_R1_Pin),
		.R2=(enum Location_Status)!GPIO_ReadInputDataBit(Sensor_GPIO_Port,Sensor_R2_Pin)
	};
	return Sensed_Location_Status;
}

ErrorInformation Sensor_GetErrorInformation(void){
	ErrorInformation errorinformation;
	status S=Sensor_Location_Status();
	uint8_t n=S.L2+S.L1+S.R1+S.R2;
	errorinformation.error=(n!=0)? (S.L2*weight[0]+S.L1*weight[1]+S.R1*weight[2]+S.R2*weight[3])/n : 0;
	errorinformation.OnLineNum=n;
	return errorinformation;
}

void Sensor_Blind(enum Sensor_List sensor){
	weight[sensor]=0;
}

void Sensor_Reset(void){
	for(uint8_t i=0;i<4;i++){
		weight[i]=Weight[i];
	}
}

