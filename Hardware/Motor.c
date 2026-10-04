#include "stm32f10x.h"
#include "Motor.h"

#define Motor_GPIO_Port_E		GPIOA		//编码器
#define Motor_GPIO_Port_C 		GPIOB		//AIN/BIN
#define Motor_GPIO_Port_P 		GPIOB		//PWM
#define Motor_GPIO_Port_S		GPIOA		//STBY
#define Motor_GPIO_CLK_E		RCC_APB2Periph_GPIOA
#define Motor_GPIO_CLK_C  		RCC_APB2Periph_GPIOB
#define Motor_GPIO_CLK_P  		RCC_APB2Periph_GPIOB
#define Motor_GPIO_CLK_S		RCC_APB2Periph_GPIOA

//PA0,PA1 TIM2 左A电机编码 PB8 TIM4 PWM
//PA6,PA7 TIM3 右B电机编码 PB9 TIM4 PWM
//PA8 STBY
#define Motor_LeftEncoder1_Pin 		GPIO_Pin_0
#define Motor_LeftEncoder2_Pin 		GPIO_Pin_1
#define Motor_LeftPWM_Pin 			GPIO_Pin_8
#define Motor_RightEncoder1_Pin 	GPIO_Pin_6
#define Motor_RightEncoder2_Pin 	GPIO_Pin_7
#define Motor_RightPWM_Pin 			GPIO_Pin_9
#define Motor_STBY_Pin				GPIO_Pin_8

//PB12,PB13 控制左电机	PB14,PB15 控制右电机
#define Motor_AIN1_Pin				GPIO_Pin_12
#define Motor_AIN2_Pin				GPIO_Pin_13
#define Motor_BIN1_Pin				GPIO_Pin_14
#define Motor_BIN2_Pin				GPIO_Pin_15

#define ARR 1000
#define PSC 72
int16_t PWM_MAX=ARR-1;

//每厘米多少个编码器计数(左右轮取平均)。标定:Config 页的"1m标定"自动改写,也可以手推100cm读数/100
float CountPerCM=76.5f;


void Motor_Control_Init(void){									
	RCC_APB2PeriphClockCmd(Motor_GPIO_CLK_C | Motor_GPIO_CLK_S,ENABLE);	
	GPIO_InitTypeDef GPIOInitStructure={
		.GPIO_Pin=Motor_AIN1_Pin | Motor_AIN2_Pin | Motor_BIN1_Pin | Motor_BIN2_Pin,
		.GPIO_Mode=GPIO_Mode_Out_PP,
		.GPIO_Speed=GPIO_Speed_50MHz
	};	
	GPIO_Init(Motor_GPIO_Port_C,&GPIOInitStructure);
	
	GPIOInitStructure.GPIO_Pin=Motor_STBY_Pin;		//STBY
	GPIO_Init(Motor_GPIO_Port_S,&GPIOInitStructure);
}

void Motor_PWM_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);		
	RCC_APB2PeriphClockCmd(Motor_GPIO_CLK_P,ENABLE);		
	GPIO_InitTypeDef GPIOInitStructure={
		.GPIO_Pin=Motor_LeftPWM_Pin | Motor_RightPWM_Pin,
		.GPIO_Mode=GPIO_Mode_AF_PP,
		.GPIO_Speed=GPIO_Speed_50MHz
	};
	GPIO_Init(Motor_GPIO_Port_P,&GPIOInitStructure);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure={
		.TIM_CounterMode=TIM_CounterMode_Up,
		.TIM_ClockDivision=TIM_CKD_DIV1,
		.TIM_Period=ARR-1,		//ARR
		.TIM_Prescaler=PSC-1,		//PSC		
		.TIM_RepetitionCounter=0
	};
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);
	
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse= 0;		//CCR
	TIM_OC3Init(TIM4,&TIM_OCInitStructure);		
	TIM_OC4Init(TIM4,&TIM_OCInitStructure);		
	TIM_Cmd(TIM4,ENABLE);
}

void Motor_Encoder_Init(void){
	RCC_APB2PeriphClockCmd(Motor_GPIO_CLK_E,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
	GPIO_InitTypeDef GPIOInitStructure={
		.GPIO_Pin=Motor_LeftEncoder1_Pin | Motor_LeftEncoder2_Pin | Motor_RightEncoder1_Pin | Motor_RightEncoder2_Pin,
		.GPIO_Mode=GPIO_Mode_IN_FLOATING,
		.GPIO_Speed=GPIO_Speed_50MHz
	};	
	GPIO_Init(Motor_GPIO_Port_E,&GPIOInitStructure);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period=65536-1;
	TIM_TimeBaseInitStructure.TIM_Prescaler=1-1;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);
	
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter=0x00;
	TIM_ICInit(TIM2,&TIM_ICInitStructure);
	TIM_ICInit(TIM3,&TIM_ICInitStructure);
	TIM_EncoderInterfaceConfig(TIM2,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);
	TIM_EncoderInterfaceConfig(TIM3,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);
}

static int32_t MileageTotal=0;						//左右轮带符号累计计数之和
static uint16_t MileageLast1=0,MileageLast2=0;

void Motor_MileageCountClean(void){
	TIM_SetCounter(TIM2,0);
	TIM_SetCounter(TIM3,0);
	MileageTotal=0;
	MileageLast1=0;
	MileageLast2=0;
}
void Motor_MileageCountSwitch(enum Switch Mileage_Switch){
	Motor_MileageCountClean();
	if (Mileage_Switch==turn_on){
		TIM_Cmd(TIM2,ENABLE);
		TIM_Cmd(TIM3,ENABLE);
	}else{
		TIM_Cmd(TIM2,DISABLE);
		TIM_Cmd(TIM3,DISABLE);	
	}
}

//瞬时原始计数(左右取平均)。标定和调试用,16位计数器回绕后值会跳变
int16_t Motor_GetMileageCount(void){
	int16_t N1=(int16_t)TIM_GetCounter(TIM2);
	int16_t N2=(int16_t)TIM_GetCounter(TIM3);
	return (int16_t)((N1+N2)/2);
}

//取增量累加,自动处理16位回绕,所以能跑任意距离
static void Motor_MileageUpdate(void){
	uint16_t now1=TIM_GetCounter(TIM2);
	uint16_t now2=TIM_GetCounter(TIM3);
	MileageTotal+=(int16_t)(now1-MileageLast1);		//差值转int16,一进位就自动借位
	MileageTotal+=(int16_t)(now2-MileageLast2);
	MileageLast1=now1;
	MileageLast2=now2;
}

//累计里程,单位cm。每个控制拍调一次(内部顺手更新计数)
float Motor_GetMileage(void){
	Motor_MileageUpdate();
	return (float)MileageTotal/2.0f/CountPerCM;
}

uint16_t myabs(int16_t n){
	return (n>=0)? (int16_t)n:(int16_t)-n;
}

void Motor_SetSpeed(int16_t left_speed,int16_t right_speed){	//绝对速度(正=前进,负=后退)
	if (left_speed>PWM_MAX) left_speed=PWM_MAX;
	if (left_speed<-PWM_MAX) left_speed=-PWM_MAX;
	if (right_speed>PWM_MAX) right_speed=PWM_MAX;
	if (right_speed<-PWM_MAX) right_speed=-PWM_MAX;

	if (left_speed>=0){			//左轮方向(前进:AIN1=1,AIN2=0)
		GPIO_SetBits(Motor_GPIO_Port_C,Motor_AIN1_Pin);
		GPIO_ResetBits(Motor_GPIO_Port_C,Motor_AIN2_Pin);
	}else{
		GPIO_SetBits(Motor_GPIO_Port_C,Motor_AIN2_Pin);
		GPIO_ResetBits(Motor_GPIO_Port_C,Motor_AIN1_Pin);
	}
	if (right_speed>=0){			//右轮方向(前进:BIN1=1,BIN2=0)
		GPIO_SetBits(Motor_GPIO_Port_C,Motor_BIN1_Pin);
		GPIO_ResetBits(Motor_GPIO_Port_C,Motor_BIN2_Pin);
	}else{
		GPIO_SetBits(Motor_GPIO_Port_C,Motor_BIN2_Pin);
		GPIO_ResetBits(Motor_GPIO_Port_C,Motor_BIN1_Pin);
	}
	TIM_SetCompare3(TIM4,myabs(left_speed));
	TIM_SetCompare4(TIM4,myabs(right_speed));
}

void Motor_Switch(enum Switch car_switch){
	if (car_switch==turn_off){
		GPIO_ResetBits(Motor_GPIO_Port_S,Motor_STBY_Pin);
	}else {
		GPIO_SetBits(Motor_GPIO_Port_S,Motor_STBY_Pin);
	}	
}

void Motor_Init(void){
	Motor_Control_Init();
	Motor_PWM_Init();
	Motor_Encoder_Init();
}

