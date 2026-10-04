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
#define NAV_ALL4_TARGET		4		//阶段1:出发点 -> C,要数几个四路全亮(实测校正)
#define NAV_DEBOUNCE_CM		2.0f	//四路全亮后前推距离:把车推出宽白区,否则下一拍会重复触发
#define NAV_STRAIGHT_CM		250.0f	//阶段2:一次直线检测跑完全程的门槛。要求"大于出环后第一段
									//      长直线、小于最后那段约3m的超长直线",这样中间路段
									//      攒不够、只有最后那段能攒够,出口就落在超长直线尽头
									//      (=停车点)。现场看第4行 VALID 实测修正
#define NAV_PARK_CM			0.0f	//阶段3:直线检测出口到停车点还要走的距离(0=就地停,待标定)

#define RUN_RES_DONE		8		//正常跑完的原因码(Act_OK_* 是 0~5,不会和它撞)

/* 阶段号:0=待机 1=数四路全亮 2=直线检测 3=定距停车 */
static uint8_t Stage=0;
static uint8_t JuncCnt=0;			//阶段1:已数到的四路全亮次数
static uint8_t RunRes=0;			//停车原因(Car_Act 的返回值,或 RUN_RES_DONE)

/* main 里只有状态机和对 Control.c 里那些函数的调用,界面/动作全在 Control.c */

int main(){
	uint8_t r=0;

	Devices_Init();
	Delay_ms(50);

	/* 上电自动陀螺仪零偏标定,要 500*10ms = 5s */
	OLED_ShowString(1,1,"Calibrating MPU");
	MPU_6050_GyroErrorCorrect();
	OLED_Clear();

	while (1){
		/* ---------- 交互界面 ----------
		   Interact() 自己跑翻页+按键循环,里面有"开始比赛"页和两个配置页。
		   返回 1 = 用户要开始比赛;返回 0 = 刚跑完一个配置功能,回界面重新选。 */
		if (Interact()==0) continue;

		/* ---------- 准备起跑 ---------- */
		Countdown_Show(3);					//3秒倒计时,结束后显示 GO!
		OLED_Clear();

		Sensor_Reset();						//清掉上一轮可能残留的传感器屏蔽状态
		Motor_Switch(turn_on);
		Motor_SetSpeed(0,0);
		Timer_Switch(turn_on);

		JuncCnt=0;
		Stage=1;
		Run_Show(Stage,JuncCnt,0,0);

		/* ================== 比赛状态机(线性) ==================
		   S 出发点 --阶段1--> C(数完四路全亮) --阶段2--> 停车点 --阶段3--> 停车

		   阶段2 只调一次 Act_Straight,就把中间整段(连续圆环 + 直角弯 + 超长直线)跑完。
		   依据:圆环和直角弯处的偏航都超过 ACT_STRAIGHT_ANGLE,有效行程会被清零重新累计,
		   所以只有最后那段约3m的超长直线能攒够 NAV_STRAIGHT_CM;攒够之后再遇到的第一个
		   特殊点(三路/四路全亮)就是停车点。每一步都是一次 Car_Act 调用,返回值统一用
		   ACT_IS_OK 判断,异常直接跳到 RUN_END 停车。 */

		/* ---------- 阶段1:S -> C,数四路全亮 ---------- */
		while (JuncCnt<NAV_ALL4_TARGET){
			r=Car_Act(Act_All4,0.0f);					//循迹,等四路全亮
			if (ACT_IS_OK(r)==0) goto RUN_END;
			r=Car_Act(Act_Distance,NAV_DEBOUNCE_CM);		//推出宽白区,免得原地重复触发
			if (ACT_IS_OK(r)==0) goto RUN_END;
			JuncCnt++;
			Run_Show(Stage,JuncCnt,0,0);
		}

		/* ---------- 阶段2:一次直线检测跑完 C -> 停车点 ---------- */
		Stage=2;
		Run_Show(Stage,JuncCnt,0,0);
		r=Car_Act(Act_Straight,NAV_STRAIGHT_CM);
		if (ACT_IS_OK(r)==0) goto RUN_END;

		/* ---------- 阶段3:停车 ---------- */
		Stage=3;
		Run_Show(Stage,JuncCnt,0,0);
		r=Car_Act(Act_Distance,NAV_PARK_CM);

	RUN_END:
		Motor_SetSpeed(0,0);
		RunRes=ACT_IS_OK(r)? RUN_RES_DONE : r;
		Run_Show(Stage,JuncCnt,RunRes,1);			//结果留在屏幕上

		Wait_Key2();								//确认后回交互界面,可以再跑一次
		OLED_Clear();
	}
}
