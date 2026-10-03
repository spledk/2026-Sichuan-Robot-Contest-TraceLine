#include "stm32f10x.h"
#include "Control.h"
#include "MPU_6050.h"
#include "OLED.h"
#include "Delay.h"
//PA0,PA1 TIM2 左A电机编码 PB8 TIM4 PWM
//PA6,PA7 TIM3 右B电机编码 PB9 TIM4 PWM
//PA2,PA3,PA4,PA5接Sensor 	PA4=L2, PA5=L1, PA3=R1, PA2=R2
//PB6,PB7 OLED CLOCK DATA
//PB12,13,14,15 AIN BIN
//PA8 STBY
//PB0,PB1 KEY1 KEY2
//PB10,PB11 MPU6050 SCL SDA


int main(){
	Devices_Init();
	Delay_ms(50);

	OLED_ShowString(1,1,"Calibrating MPU");       // 零偏标定要 500*10ms = 5s
	MPU_6050_GyroErrorCorrect();
	OLED_Clear();

	Interact();                                   // 菜单 + 四种比赛模式,不会返回
	while (1);
}
