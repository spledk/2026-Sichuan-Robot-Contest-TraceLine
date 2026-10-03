#include "stm32f10x.h"
#include "MyI2C_Software.h"
#include "Delay.h"

#define I2C_GPIO_Port GPIOB
#define I2C_GPIO_CLK RCC_APB2Periph_GPIOB
#define I2C_SCL_Pin GPIO_Pin_10
#define I2C_SDA_Pin GPIO_Pin_11

#define MyI2C_SCL_WriteBit(x)	\
{	\
	GPIO_WriteBit(I2C_GPIO_Port,I2C_SCL_Pin,(BitAction)(x));	\
	Delay_us(10);	\
}
#define MyI2C_SDA_WriteBit(x)	\
{	\
	GPIO_WriteBit(I2C_GPIO_Port,I2C_SDA_Pin,(BitAction)(x));	\
	Delay_us(10);	\
}
//x一定要带括号，不然宏会直接将原表达式替换x

//software I2C//

enum ACKAnswer MyI2C_ReceiveACK(void);

void MyI2C_Init(void){
	RCC_APB2PeriphClockCmd(I2C_GPIO_CLK,ENABLE);
	GPIO_InitTypeDef G_IS;
	G_IS.GPIO_Mode=GPIO_Mode_Out_OD;
	G_IS.GPIO_Pin=I2C_SCL_Pin | I2C_SDA_Pin;
	G_IS.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&G_IS);
	MyI2C_SCL_WriteBit(1);
	MyI2C_SDA_WriteBit(1);
}



void MyI2C_Start(void){
	MyI2C_SDA_WriteBit(1);		//SDA上一位不确定,兼容继续
	MyI2C_SCL_WriteBit(1);
	MyI2C_SDA_WriteBit(0);
	MyI2C_SCL_WriteBit(0);		//拉低SCL方便发数据
}

enum ACKAnswer MyI2C_SendAddress(uint8_t address,enum I2C_Mode mode){
	return MyI2C_SendByte(address<<1 | mode);
}

void MyI2C_Stop(void){
	MyI2C_SDA_WriteBit(0);
	MyI2C_SCL_WriteBit(1);
	MyI2C_SDA_WriteBit(1);
}

uint8_t MyI2C_SDA_ReadBit(void){			
	return GPIO_ReadInputDataBit(GPIOB,I2C_SDA_Pin);
}

enum ACKAnswer MyI2C_ReceiveACK(void){
	int16_t retry=100;
	MyI2C_SDA_WriteBit(1);
	MyI2C_SCL_WriteBit(1);
	while(MyI2C_SDA_ReadBit()!=0 && retry>0) retry--; 
	enum ACKAnswer ACK=(enum ACKAnswer)MyI2C_SDA_ReadBit();
	MyI2C_SCL_WriteBit(0);
	return ACK;
}

void MyI2C_SendACK(void){
	MyI2C_SDA_WriteBit(0);
	MyI2C_SCL_WriteBit(1);
	MyI2C_SCL_WriteBit(0);
}

void MyI2C_SendNACK(void){
	MyI2C_SDA_WriteBit(1);
	MyI2C_SCL_WriteBit(1);
	MyI2C_SCL_WriteBit(0);
}

enum ACKAnswer MyI2C_SendByte(uint8_t Byte){
	for(int8_t i=7;i>=0;i--){
		MyI2C_SDA_WriteBit(Byte>>i & 0x01);		
		MyI2C_SCL_WriteBit(1);
		MyI2C_SCL_WriteBit(0);
	}
	return MyI2C_ReceiveACK();
}

uint8_t MyI2C_ReceiveByte(void){
	uint8_t Byte=0x00;
	MyI2C_SDA_WriteBit(1);
	for(int8_t i=7;i>=0;i--){
		MyI2C_SCL_WriteBit(1);
		Byte|=MyI2C_SDA_ReadBit()<<i;
		MyI2C_SCL_WriteBit(0);
	}
	return Byte;
}



