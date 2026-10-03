#include "stm32f10x.h"
#include "MPU_6050.h"
#include "MPU6050_Reg.h"
#include "MyI2C_Software.h"
#include "Delay.h"
#include "Timer.h"

#define MPU_6050_Address 0x68

static float YawAngle=0.0f;
static uint16_t YawLastTime=0;		//上一次积分时的 Timer 计数

void MPU_6050_WriteReg(uint8_t RegAddress,uint8_t Data){
	MyI2C_Start();
	if(MyI2C_SendAddress(MPU_6050_Address,Write)==Answered){};
	if(MyI2C_SendByte(RegAddress)==Answered){};
	if (MyI2C_SendByte(Data)==Answered){};
	MyI2C_Stop();
}
uint8_t MPU_6050_ReadReg(uint8_t RegAddress){
	MyI2C_Start();
	if(MyI2C_SendAddress(MPU_6050_Address,Write)==Answered){};
	if(MyI2C_SendByte(RegAddress)==Answered){};
	
	MyI2C_Start();
	if(MyI2C_SendAddress(MPU_6050_Address,Read)==Answered){};
	uint8_t Byte=MyI2C_ReceiveByte();
	MyI2C_SendNACK();
	MyI2C_Stop();
	return Byte;
}

void MPU_6050_Init(void){ 
	MyI2C_Init();
	MPU_6050_WriteReg(MPU6050_PWR_MGMT_1,0x01);
	MPU_6050_WriteReg(MPU6050_PWR_MGMT_2,0x00);
	MPU_6050_WriteReg(MPU6050_SMPLRT_DIV,0x09);
	MPU_6050_WriteReg(MPU6050_CONFIG,0x06);
	MPU_6050_WriteReg(MPU6050_GYRO_CONFIG,0x00);
	MPU_6050_WriteReg(MPU6050_ACCEL_CONFIG,0x00);
	MPU_6050_WriteReg(MPU6050_INT_ENABLE,0x01);
}

static int16_t GyroError[3]={0};		//x,y,z

void MPU_6050_GyroErrorCorrect(void){
	uint16_t N=500;
	int32_t gyroerror[3]={0};
	for(uint16_t i=1;i<=N;i++){
		MyI2C_Start();
		if(MyI2C_SendAddress(MPU_6050_Address,Write)==Answered){};
		if(MyI2C_SendByte(MPU6050_GYRO_XOUT_H)==Answered){};
		MyI2C_Start();
		if(MyI2C_SendAddress(MPU_6050_Address,Read)==Answered){};
		uint8_t data[6]={0};
		for(uint8_t i=0;i<5;i++){
			data[i]=MyI2C_ReceiveByte();
			MyI2C_SendACK();
		}
		data[5]=MyI2C_ReceiveByte();
		MyI2C_SendNACK();
		MyI2C_Stop();
		
		for(uint8_t i=0;i<3;i++){
			gyroerror[i]+=(int16_t)(data[i*2]<<8 | data[i*2+1]);
		}
		Delay_ms(10);
	}
	for(uint8_t i=0;i<3;i++){
		GyroError[i]=gyroerror[i]/N;
	}
}

struct MPU_6050_Data MPU_6050_GetData(void){
	struct MPU_6050_Data Data={0};
	uint8_t data[14]={0};
	
	MyI2C_Start();
	if(MyI2C_SendAddress(MPU_6050_Address,Write)==Answered){};
	if(MyI2C_SendByte(MPU6050_ACCEL_XOUT_H)==Answered){};
	
	MyI2C_Start();
	if(MyI2C_SendAddress(MPU_6050_Address,Read)==Answered){};
	for(uint8_t i=0;i<13;i++){	
	data[i]=MyI2C_ReceiveByte();
	MyI2C_SendACK();
	}
	data[13]=MyI2C_ReceiveByte();
	MyI2C_SendNACK();
	MyI2C_Stop();
	Data.AccX=data[0]<<8 | data[1];
	Data.AccY=data[2]<<8 | data[3];
	Data.AccZ=data[4]<<8 | data[5];		//data[6],data[7]为温度
	Data.GyroX=(int16_t)(data[8]<<8 | data[9])-GyroError[0];
	Data.GyroY=(int16_t)(data[10]<<8 | data[11])-GyroError[1];
	Data.GyroZ=(int16_t)(data[12]<<8 | data[13])-GyroError[2];
	return Data;
}

struct MPU_6050_TurnNeedData MPU_6050_GetTurnNeedData(void){
	struct MPU_6050_TurnNeedData Data={Old,0};
	int16_t High=0,Low=0;
	
	MyI2C_Start();
	if(MyI2C_SendAddress(MPU_6050_Address,Write)==Answered){};
	if(MyI2C_SendByte(MPU6050_INT_STATUS)==Answered){};
	MyI2C_Start();
	if(MyI2C_SendAddress(MPU_6050_Address,Read)==Answered){};
	Data.IsNew=(enum Data_Status)(MyI2C_ReceiveByte() & 0x01);
	MyI2C_SendNACK();
	MyI2C_Stop();
		
	MyI2C_Start();
	if(MyI2C_SendAddress(MPU_6050_Address,Write)==Answered){};
	if(MyI2C_SendByte(MPU6050_GYRO_ZOUT_H)==Answered){};
	MyI2C_Start();
	if(MyI2C_SendAddress(MPU_6050_Address,Read)==Answered){};
	High=MyI2C_ReceiveByte();
	MyI2C_SendACK();
	Low=MyI2C_ReceiveByte();
	MyI2C_SendNACK();
	MyI2C_Stop();
		
	Data.GyroZ=(int16_t)(High<<8 | Low)-GyroError[2];

	return Data;
}

void MPU_6050_YawAngle_Update(const struct MPU_6050_TurnNeedData *Data){
	uint16_t now;
	float dt;

	if (Data->IsNew!=New) return;					//没新样本:不积分,也不动时间戳

	now=Timer_GetMicros();
	dt=(float)(uint16_t)(now-YawLastTime)*1e-6f;	//两次样本之间的真实间隔,单位秒
	YawLastTime=now;

	//间隔离谱(刚上电/很久没调)就退回标称值。TIM1 每 65.5ms 回绕一次,
	//超过量程的间隔会"看起来"很短,所以这个兜底不能省
	if (dt>0.05f||dt<0.001f) dt=MPU_6050_SAMPLE_DT;

	YawAngle+=(float)Data->GyroZ/MPU_6050_GYRO_SENS*dt;
}

float MPU_6050_GetYaw(void){
	return YawAngle;
}

void MPU_6050_SetYaw(float targetdegree){
	YawAngle=targetdegree;
	YawLastTime=Timer_GetMicros();	//时间戳同步到现在,否则下一拍的 dt 是假的
}

