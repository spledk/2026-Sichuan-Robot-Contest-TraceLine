#include "stm32f10x.h"
#include "Timer.h"

void Timer_Init(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure={
		.TIM_Prescaler=72-1,			//1µs
		.TIM_Period=65536-1,			//
		.TIM_CounterMode=TIM_CounterMode_Up,
		.TIM_ClockDivision=TIM_CKD_DIV1,
		.TIM_RepetitionCounter=0
	};
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStructure);
}


void Timer_Switch(enum Switch Timer_State){
	TIM_SetCounter(TIM1,0);
	if (Timer_State==turn_on) TIM_Cmd(TIM1,ENABLE);
	else TIM_Cmd(TIM1,DISABLE);
}

uint16_t Timer_GetMicros(void){
	return (uint16_t)TIM_GetCounter(TIM1);
}

float Timer_GetSeconds(void){
	return (float)Timer_GetMicros()*1e-6f;
}
