#include "stm32f10x.h"
#include "Sensor.h"

#define Sensor_GPIO_Port GPIOA
#define Sensor_GPIO_CLK  RCC_APB2Periph_GPIOA

//传感器接口非顺序排列
#define Sensor_L2_Pin GPIO_Pin_4		
#define Sensor_L1_Pin GPIO_Pin_5		
#define Sensor_R1_Pin GPIO_Pin_3		
#define Sensor_R2_Pin GPIO_Pin_2	

//现实测量值 单位cm  顺序 L2,L1,R1,R2  基准表,不要改
const float Weight[4]={-3.15,-0.35,0.35,3.15};
//传感器开关位: 1=参与运算, 0=被 Sensor_Blind 屏蔽
uint8_t Sensor_Enable[4]={1,1,1,1};

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
	uint8_t on[4]={S.L2,S.L1,S.R1,S.R2};
	uint8_t n=0;
	float sum=0.0f;
	for(uint8_t i=0;i<4;i++){
		if(Sensor_Enable[i]==0) continue;		//被屏蔽的传感器整项不参与
		n+=on[i];
		sum+=on[i]*Weight[i];
	}
	errorinformation.error=(n!=0)? sum/n : 0;
	errorinformation.OnLineNum=n;
	return errorinformation;
}

void Sensor_Blind(enum Sensor_List sensor){
	Sensor_Enable[sensor]=0;
}

void Sensor_Reset(void){
	for(uint8_t i=0;i<4;i++){
		Sensor_Enable[i]=1;
	}
}
