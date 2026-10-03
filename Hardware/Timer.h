#ifndef Timer_H
#define Timer_H
#include "stm32f10x.h"
#include "Motor.h"		

void Timer_Init(void);								
void Timer_Switch(enum Switch Timer_State);			
uint16_t Timer_GetMicros(void);						
float Timer_GetSeconds(void);						

#endif
