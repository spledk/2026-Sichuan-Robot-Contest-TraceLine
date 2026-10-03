#ifndef MyI2C_Software_H
#define MyI2C_Software_H
#include "stm32f10x.h"

enum ACKAnswer{
	Answered=0,
	NotAnswered
};
enum I2C_Mode{
	Write=0,
	Read
};

void MyI2C_Init(void);

void MyI2C_Start(void);
void MyI2C_Stop(void);

enum ACKAnswer MyI2C_SendAddress(uint8_t address,enum I2C_Mode mode);
enum ACKAnswer MyI2C_SendByte(uint8_t Byte);

uint8_t MyI2C_ReceiveByte(void);

void MyI2C_SendACK(void);
void MyI2C_SendNACK(void);

#endif
