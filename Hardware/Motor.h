#ifndef Motor_H
#define Motor_H
#include "stm32f10x.h"
enum Switch{
	turn_off=0,
	turn_on
};


extern int16_t PWM_MAX;

void Motor_Init(void);
void Motor_Switch(enum Switch);		
void Motor_SetSpeed(int16_t left_speed,int16_t right_speed);
void Motor_MileageCountClean(void);
void Motor_MileageCountSwitch(enum Switch Mileage_Switch);
int16_t Motor_GetMileageCount(void);
float Motor_GetMileage(void);

#endif
