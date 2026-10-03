#include "stm32f10x.h"
#include "Key.h"
#include "Delay.h"

//PB0,PB1 KEY1 KEY2
#define Key_GPIO_Port GPIOB
#define Key_GPIO_CLK RCC_APB2Periph_GPIOB
#define Key_GPIO_Pin1 GPIO_Pin_0
#define Key_GPIO_Pin2 GPIO_Pin_1

#define Key1_Get GPIO_ReadInputDataBit(Key_GPIO_Port,Key_GPIO_Pin1)
#define Key2_Get GPIO_ReadInputDataBit(Key_GPIO_Port,Key_GPIO_Pin2)

void Key_Init(void){
	RCC_APB2PeriphClockCmd(Key_GPIO_CLK, ENABLE);
	
	GPIO_InitTypeDef GPIOInitStruct={
		.GPIO_Pin=Key_GPIO_Pin1 | Key_GPIO_Pin2,
		.GPIO_Mode=GPIO_Mode_IPD,
		.GPIO_Speed=GPIO_Speed_50MHz
	};
	GPIO_Init(Key_GPIO_Port,&GPIOInitStruct);	
	
}

enum Key_Status Key1_GetStatus(void){
	if (Key1_Get==1){
		Delay_ms(100);
		while (Key1_Get==1);
		Delay_ms(100);
		return Press;
	}else {
		return Release;
	}
}

enum Key_Status Key2_GetStatus(void){
	if (Key2_Get==1){
		Delay_ms(100);
		while (Key2_Get==1);
		Delay_ms(100);
		return Press;
	}else {
		return Release;
	}
}
