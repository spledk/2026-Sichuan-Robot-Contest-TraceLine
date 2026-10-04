#include "stm32f10x.h"
#include "Control.h"
#include "Motor.h"
#include "Sensor.h"
#include "Timer.h"
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

/* ================== 比赛状态机参数(全部现场标定) ================== */
#define NAV_ALL4_TARGET		4		//阶段1:出发点 -> C,要数几个四路全亮(2个十字路?实测校正)
#define NAV_DEBOUNCE_CM		2.0f	//四路全亮后前推距离:把车推出宽白区,否则下一拍会重复触发
#define NAV_STRAIGHT_CM		220.0f	//阶段2:C -> F 这段长直线上要攒够的直线有效行程(图标2824,留余量)
#define NAV_CORNER_TARGET	1		//阶段3:F -> G 还要找几个直角弯(F 已经在阶段2 认掉了)
#define NAV_DEBOUNCE3_CM	8.0f	//三路全亮后前推距离:走过这个直角弯,避免重复计数
#define NAV_PARK_CM			0.0f	//阶段4:最后一个直角弯之后到停车点的定距距离(0=就地停,待标定)

#define RUN_RES_DONE		8		//正常跑完的原因码(Act_OK_* 是 0~3,不会和它撞)

/* 阶段号:0=待机 1=数四路全亮 2=直线检测 3=找直角弯 4=定距停车 */
static uint8_t Stage=0;
static uint8_t JuncCnt=0;			//阶段1:已数到的四路全亮次数
static uint8_t CornrCnt=0;			//阶段3:已找到的直角弯个数
static uint8_t RunRes=0;			//停车原因(Car_Act 的返回值,或 RUN_RES_DONE)

static void Run_Show(uint8_t stopped){
	OLED_ShowString(1,1,stopped? "2026 STOP ":"2026 TRACE");	//STOP 后面补空格,盖掉 TRACE 的尾巴
	OLED_ShowString(2,1,"STAGE:");
	OLED_ShowNum(2,7,Stage,1);
	OLED_ShowString(3,1,"JUNC :");
	OLED_ShowNum(3,7,JuncCnt,2);
	OLED_ShowString(4,1,"CORNR:");
	OLED_ShowNum(4,7,CornrCnt,1);
	if (stopped){					//停机后在第二行右边补一个原因码
		OLED_ShowString(2,9,"R=");
		OLED_ShowNum(2,11,RunRes,1);
	}
}

int main(){
	uint8_t r=0;

	Devices_Init();
	Delay_ms(50);

	OLED_ShowString(1,1,"Calibrating MPU");       // 零偏标定要 500*10ms = 5s
	MPU_6050_GyroErrorCorrect();
	OLED_Clear();

	Interact();                                   // 翻页,在可以跑的页上按 K2 之后返回

	/* ================== 比赛状态机(线性) ==================
	   S 出发点 --阶段1--> C(数完四路全亮,进入连续圆环) --阶段2--> F(出环,长直线尽头)
	          --阶段3--> G(直角弯走完) --阶段4--> 停车点
	   每一步都是一次 Car_Act 调用,返回值统一用 ACT_IS_OK 判断,异常直接跳到 RUN_END 停车。 */
	Sensor_Reset();					//清掉上一轮可能残留的传感器屏蔽状态
	Motor_Switch(turn_on);
	Motor_SetSpeed(0,0);
	Timer_Switch(turn_on);
	Stage=1;
	Run_Show(0);

	/* ---------- 阶段1:S -> C,数四路全亮 ---------- */
	while (JuncCnt<NAV_ALL4_TARGET){
		r=Car_Act(Act_All4,0.0f);					//循迹,等四路全亮
		if (ACT_IS_OK(r)==0) goto RUN_END;
		r=Car_Act(Act_Distance,NAV_DEBOUNCE_CM);		//推出宽白区,免得原地重复触发
		if (ACT_IS_OK(r)==0) goto RUN_END;
		JuncCnt++;
		Run_Show(0);
	}

	/* ---------- 阶段2:C -> F,直线检测(攒够直线 + 三路全亮) ---------- */
	Stage=2;
	Run_Show(0);
	r=Car_Act(Act_Straight,NAV_STRAIGHT_CM);
	if (ACT_IS_OK(r)==0) goto RUN_END;

	/* ---------- 阶段3:F -> G,继续找直角弯 ---------- */
	Stage=3;
	Run_Show(0);
	while (CornrCnt<NAV_CORNER_TARGET){
		r=Car_Act(Act_All3,0.0f);					//循迹,等三路全亮
		if (ACT_IS_OK(r)==0) goto RUN_END;
		r=Car_Act(Act_Distance,NAV_DEBOUNCE3_CM);	//走过这个直角弯
		if (ACT_IS_OK(r)==0) goto RUN_END;
		CornrCnt++;
		Run_Show(0);
	}

	/* ---------- 阶段4:G -> 停车点,定距循迹 ---------- */
	Stage=4;
	Run_Show(0);
	r=Car_Act(Act_Distance,NAV_PARK_CM);

RUN_END:
	Motor_SetSpeed(0,0);
	RunRes=ACT_IS_OK(r)? RUN_RES_DONE : r;
	Run_Show(1);
	while (1);
}
