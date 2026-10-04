#include "stm32f10x.h"
#include "Control.h"
#include "Motor.h"
#include "Sensor.h"
#include "Key.h"
#include "MPU_6050.h"
#include "OLED.h"
#include "Interface.h"
#include "Delay.h"
#include "Timer.h"

//**比赛流程**//
/*	*
大致分为三个阶段：1.进入连续圆环前2.连续圆环3.退出连续圆环后
其中1,3不难，重点是1进2的判断，2中的应对措施，2进3的判断
目前主要有两种方法：1.四路特殊点判断2.陀螺仪检测
实际可能二者结合
**/

void Devices_Init(void){
	Timer_Init();
	OLED_Init();
	Motor_Init();
	Sensor_Init();
	MPU_6050_Init();	
}

//**交互界面**//
/*	*
整个界面集中在这里,main 只调用 Interact() 和 Run_Show()。
页表在 Interface.h,按键:K1 = 顺页翻页,K2 = 执行当前页。
Key_GetStatus() 本身就是"按下并等松手"的,所以不会连触发。

Interact() 只有两种返回:
	1 = 用户在"开始比赛"页按了 K2,main 去跑比赛
	0 = 用户在配置页跑完一个功能,main 直接 continue 再进来
**/
#define ACT_CALIB_CM			100.0f	//1m 标定用的实际线长(cm)
#define ACT_CALIB_APPROACH_CM	40.0f	//1m 标定:起步直行,最多走这么远去找线

static enum Interface_Page CurrentPage=Page_Run;

/* 定长写一行:补满 16 个字符,换页时不会残留上一页的字 */
static void Show_Line(uint8_t Line,const char *s){
	char buf[17];
	uint8_t i=0,j;

	while (i<16 && s[i]!='\0'){ buf[i]=s[i]; i++; }
	for (j=i;j<16;j++) buf[j]=' ';
	buf[16]='\0';
	OLED_ShowString(Line,1,buf);
}

static void Page_Show(void){
	switch (CurrentPage){
		case Page_Run:
			Show_Line(1,"2026 TRACE");
			Show_Line(2,"K2: START RUN");
			Show_Line(3,"K1: NEXT PAGE");
			Show_Line(4,"");
			break;
		case Page_Gyro:
			Show_Line(1,"CONFIG 1/2");
			Show_Line(2,"GYRO CALIB");
			Show_Line(3,"CAR STILL, HANDS OFF");
			Show_Line(4,"K2:RUN  K1:NEXT");
			break;
		case Page_Meter:
			Show_Line(1,"CONFIG 2/2");
			Show_Line(2,"1M CALIB");
			Show_Line(3,"CAR BEFORE 1M LINE");
			Show_Line(4,"K2:RUN  K1:NEXT");
			break;
		default:
			break;
	}
}

/* 第4行上的秒倒计时,数完显示 GO! */
void Countdown_Show(uint8_t sec){
	Show_Line(4,"GO IN:");
	OLED_ShowNum(4,7,sec,1);
	while (sec>0){
		Delay_ms(1000);
		sec--;
		if (sec>0) OLED_ShowNum(4,7,sec,1);
	}
	Show_Line(4,"GO!");
}

void Wait_Key2(void){
	while (Key2_GetStatus()!=Press);
}

void Gyro_Calib(void){
	Show_Line(1,"CONFIG 1/2");
	Show_Line(2,"GYRO CALIB");
	Show_Line(3,"KEEP STILL...");
	Countdown_Show(3);
	Show_Line(3,"CALIBRATING...");
	MPU_6050_GyroErrorCorrect();
	Show_Line(1,"GYRO CALIB DONE");
	Show_Line(3,"K2: OK");
	Wait_Key2();
}

static float Act_CalibPulse=0.0f;		//1m 标定测到的脉冲数,只给显示用

void Meter_Calib(void){
	uint8_t r=0;

	Act_CalibPulse=0.0f;
	Show_Line(1,"CONFIG 2/2");
	Show_Line(2,"1M CALIB");
	Show_Line(3,"KEEP STILL...");
	Countdown_Show(3);

	Show_Line(1,"1M CALIB");
	Show_Line(3,"APPROACH LINE...");		//直行到压上线就返回
	Car_MoveForward(ACT_CALIB_APPROACH_CM);

	Show_Line(3,"MEASURING...");			//循迹,一脱线就用走过的里程反推
	r=Car_Act(Act_Calib,0.0f);

	if (r!=Act_OK_Calib){					//没走到线的尽头(中途脱线超时/堵转)
		Show_Line(1,"1M CALIB FAIL");
		Show_Line(2,"ERR CODE:");
		OLED_ShowNum(2,11,r,1);
		Show_Line(3,"CHECK THE 1M LINE");
		Show_Line(4,"K2: OK");
		Wait_Key2();
		return;
	}

	Show_Line(1,"1M CALIB DONE");
	Show_Line(2,"PULSE/100CM:");
	OLED_ShowNum(2,13,(uint32_t)Act_CalibPulse,4);
	Show_Line(3,"CPC x10:");
	OLED_ShowNum(3,9,(uint32_t)(CountPerCM*10.0f),4);
	Show_Line(4,"K2: OK");
	Wait_Key2();
}

uint8_t Interact(void){
	Motor_Switch(turn_on);
	Motor_SetSpeed(0,0);
	CurrentPage=Page_Run;
	Page_Show();

	while (1){
		if (Key1_GetStatus()==Press){		//K1:顺页翻页,翻过最后一页回首页
			CurrentPage++;
			if (CurrentPage>=Page_Count) CurrentPage=Page_Run;
			Page_Show();
		}
		if (Key2_GetStatus()==Press){		//K2:执行当前页
			switch (CurrentPage){
				case Page_Run:
					return 1;				//去跑比赛
				case Page_Gyro:
					Gyro_Calib();
					Page_Show();
					return 0;
				case Page_Meter:
					Meter_Calib();
					Page_Show();
					return 0;
				default:
					break;
			}
		}
	}
}

/* 状态机显示:stopped=1 时第一行变 STOP,第二行右侧加停车原因码 */
void Run_Show(uint8_t stage,uint8_t junc,uint8_t res,uint8_t stopped){
	Show_Line(1,stopped?"2026 STOP":"2026 TRACE");
	Show_Line(2,"STAGE:");
	OLED_ShowNum(2,7,stage,1);
	if (stopped){
		OLED_ShowString(2,9,"R=");
		OLED_ShowNum(2,11,res,1);
	}
	Show_Line(3,"JUNC :");
	OLED_ShowNum(3,7,junc,2);
	Show_Line(4,"VALID:");
	OLED_ShowNum(4,7,(uint32_t)Act_StraightValid,3);
}

//**集成动作函数**//
/*	*
把"循迹 / 定距 / 陀螺仪参考"集成到一个循环里:一次调用只做一件事,做到就返回,
由上位状态机串联。比赛工程只有这一个执行函数,不再考虑复用和别的用途。

  mode:
    Act_All4      循迹,遇到四路全亮返回(十字路口/斑马线)
    Act_All3      循迹,遇到三路全亮返回(直角弯)
    Act_Distance  定距循迹:循迹过程中累计里程,到 distance cm 返回(不用陀螺仪)
    Act_Straight  直线检测:循迹 + 陀螺仪。相对本段起点角度偏差不超过
                  ACT_STRAIGHT_ANGLE 的行程才算"有效行程",偏差超限就清零、以当前
                  角度为新起点重新累计;有效行程攒够 distance 之后,再遇到三路/四路
                  全亮就返回。圆环和直角弯的偏航都超容差,有效行程会被清零重来,
                  所以能筛掉中间那些不长不直的路段,只认最后那段超长直线。
  distance 只在 Act_Distance / Act_Straight 里有用,其它模式传 0。

四路/三路全亮的几何依据(线宽2.5cm,最外侧L2~R2间距6.3cm):
  四路全亮 <=> 车轴这6.3cm整条落在白区里 <=> 白区里存在一条>=6.3cm的弦。
  - 直角弯:白区是两条2.5cm条带的L形并集,斜穿时弦长=2.5/sin(phi),
    phi=45度取最大 2.5*1.414=3.54cm < 6.3cm => 直角弯最多3路,不可能给4路。
  - 十字路:两条正交条带、双向延伸,条带垂直于车身时整条车轴都在白区 => 稳给4路。
  - 一般交叉夹角a:条带覆盖车轴的条件 6.3*|cos a| <= 2.5 => a >= 66度才给4路。
  - 连续圆环的相切处:两条2.5cm线并排约5cm < 6.3cm => 也给不了4路,天然隐形。
反面:车在直线上相对赛道偏航超过 asin(2.5/6.3)=23度时,单根白线也能点亮四路。
  真路口给的是"一段"全亮(350PWM下约6~12拍),偏航造成的假全亮只1~3拍。
**/
#define ACT_PERIOD_US		10000	//控制周期 10ms
#define ACT_KP				150.0f
#define ACT_KD				50.0f
#define ACT_BASE_PWM		350		//循迹基准速度
#define ACT_STEER_MAX		400.0f	//转向限幅
#define ACT_BLIND_ERROR		5.0f	//脱线时按上次方向打死
#define ACT_BLIND_MAX		200		//连续脱线2s判脱线
#define ACT_STALL_MAX		50		//连续0.5s编码器无变化判卡死
#define ACT_MPU_MAX			100		//连续1s陀螺无新数据判失灵(只在Act_Straight生效)

#define ACT_STRAIGHT_ANGLE	10.0f	//直线检测的角度容差(度)
#define ACT_ALL3_CONFIRM	1		//三路全亮要连续这么多拍才算数;3路若提前/重复触发就改成2~3

#define MyABS(x)			((x)<0 ? -(x) : (x))

float Act_StraightValid=0.0f;			//Act_Straight 当前累计的直线有效行程(cm)
#define ACT_SHOW_TICK		20		//每20拍(200ms)把有效行程刷到OLED第4行,现场标定用

uint8_t Car_Act(enum Act_Mode mode,float distance){
	ErrorInformation Info;
	struct MPU_6050_TurnNeedData Data;
	float error=0.0f,error_last=0.0f,steer=0.0f;
	float yaw=0.0f,yaw_ref=0.0f,valid=0.0f;
	float mileage=0.0f,mileage_last=0.0f,dm=0.0f;
	int8_t dir=1;						//1=右 -1=左
	uint8_t result=Act_OK_Dist;			//占位,循环里必被覆盖
	uint16_t blind=0,stall=0,mpu_lost=0,all3=0,show_tick=0;
	int16_t left=0,right=0;

	Motor_SetSpeed(0,0);
	Motor_MileageCountSwitch(turn_on);	//里程清零并开始计数
	mileage_last=Motor_GetMileage();	//刚清零,应该是0
	MPU_6050_GetTurnNeedData();
	MPU_6050_SetYaw(0.0f);				//本段起点,直线判断以它为参考
	yaw_ref=0.0f;
	Timer_Switch(turn_on);

	while (1){
		if (Timer_GetMicros()<ACT_PERIOD_US) continue;
		Timer_Switch(turn_on);

		/* ---- 陀螺仪:每拍更新一次,给直线判断提供参考 ---- */
		Data=MPU_6050_GetTurnNeedData();
		if (Data.IsNew==New){
			MPU_6050_YawAngle_Update(&Data);
			mpu_lost=0;
		}else if (++mpu_lost>ACT_MPU_MAX){
			if (mode==Act_Straight){ result=Act_ErrMPU; break; }
		}
		yaw=MPU_6050_GetYaw();

		/* ---- 里程:本拍增量 + 防卡死 ---- */
		mileage=Motor_GetMileage();
		if (mileage==mileage_last){					//编码器一动不动
			dm=0.0f;
			if (++stall>ACT_STALL_MAX){ result=Act_ErrStall; break; }
		}else{
			dm=mileage-mileage_last;
			mileage_last=mileage;
			stall=0;
		}

		/* ---- 传感器 ---- */
		Info=Sensor_GetErrorInformation();

		if (Info.OnLineNum==4){						//四路全亮
			if (mode==Act_All4){ result=Act_OK_All4; break; }
			all3=0;									//其它模式当成宽白区,照常循迹
			blind=0;
			error=Info.error;
			if (error>0.1f) dir=1;
			else if (error<-0.1f) dir=-1;
		}else if (Info.OnLineNum==3&&mode==Act_All3){	//三路全亮(只有找直角弯时算数)
			if (++all3>=ACT_ALL3_CONFIRM){ result=Act_OK_All3; break; }
			blind=0;
			error=Info.error;
			if (error>0.1f) dir=1;
			else if (error<-0.1f) dir=-1;
		}else if (Info.OnLineNum==0){				//脱线打死
			all3=0;
			if (mode==Act_Calib){					//1m 标定:线到头了,就地反推 CountPerCM
				Act_CalibPulse=mileage*CountPerCM;	//走过的原始脉冲数(先用旧比例算回来)
				if (Act_CalibPulse>1.0f) CountPerCM=Act_CalibPulse/ACT_CALIB_CM;
				result=Act_OK_Calib;
				break;
			}
			if (++blind>ACT_BLIND_MAX){ result=Act_ErrLost; break; }
			error=dir*ACT_BLIND_ERROR;
		}else{										//正常压线
			all3=0;
			blind=0;
			error=Info.error;
			if (error>0.1f) dir=1;
			else if (error<-0.1f) dir=-1;
		}

		/* ---- 里程类模式的成功判定 ---- */
		if (mode==Act_Distance){
			if (mileage>=distance){ result=Act_OK_Dist; break; }
		}else if (mode==Act_Straight){
			if (MyABS(yaw-yaw_ref)>ACT_STRAIGHT_ANGLE){
				yaw_ref=yaw;						//超限:换起点,有效行程清零重来
				valid=0.0f;
			}else{
				valid+=dm;							//在容差内:本拍行程计入有效行程
			}
			/* 本工程不需要直线检测四路,默认只认三路:
			   直线有效行程攒够 distance 之后,再遇到三路全亮就返回 —— 出口就是 F 那个直角弯。
			   正着循迹时几何上给不了三路(要车与线夹角<=45.6度),所以这个出口很干净。 */
			Act_StraightValid=valid;				//暴露给 OLED,现场标定 NAV_STRAIGHT_CM 用
			if (++show_tick>=ACT_SHOW_TICK){		//每200ms刷一次第4行
				show_tick=0;
				OLED_ShowString(4,1,"VALID:");
				OLED_ShowNum(4,7,(uint32_t)valid,3);
			}
			/* 攒够直线之后的第一个特殊点就是出口。三路(直角弯)和四路(十字路/路口)
			   都收,省得纠结尽头到底是哪一种。 */
			if (valid>=distance&&Info.OnLineNum>=3){ result=Act_OK_Straight; break; }
		}

		/* ---- 循迹输出 ---- */
		steer=ACT_KP*error+ACT_KD*(error-error_last);
		error_last=error;

		if (steer>ACT_STEER_MAX) steer=ACT_STEER_MAX;
		if (steer<-ACT_STEER_MAX) steer=-ACT_STEER_MAX;

		left=ACT_BASE_PWM+(int16_t)steer;
		right=ACT_BASE_PWM-(int16_t)steer;

		Motor_SetSpeed(left,right);
	}
	Motor_SetSpeed(0,0);
	return result;
}

//**定距直行**//
#define MOVE_PERIOD_US	10000	//控制周期 10ms
#define MOVE_BASE_PWM	350		
#define MOVE_SLOW_PWM	200		//提前减速
#define MOVE_SLOW_ZONE_CM	10.0f	//还剩这么多 cm 就切到 MOVE_SLOW_PWM(为了停准)
#define MOVE_KP			8.0f	
#define MOVE_KD			3.0f	
#define MOVE_STEER_MAX	250		
#define MOVE_YAW_LIMIT	30.0f	//意外事件导致偏转较大直接放弃
#define MOVE_MPU_MAX	100		//陀螺失联
#define MOVE_STALL_MAX	50		//堵转:连续0.5s编码器无变化就退出

void Car_MoveForward(float distance_cm){
	float mag=MyABS(distance_cm);				//要走多少 cm(不看符号)
	float remain=0.0f;
	struct MPU_6050_TurnNeedData Data;
	float yaw=0.0f,u=0.0f,mileage=0.0f;
	float mileage_last=-1.0f;				//上一拍的里程(判堵转)
	uint16_t mpu_lost=0,stall=0;
	int16_t steer=0,base=0;
	int8_t dir=(distance_cm<0.0f)? -1:1;		//正=前进 负=倒车

	if (mag<0.5f) return;

	Motor_SetSpeed(0,0);
	Motor_MileageCountSwitch(turn_on);		
	MPU_6050_GetTurnNeedData();				
	MPU_6050_SetYaw(0.0f);					
	Timer_Switch(turn_on);					

	while (1){
		if (Timer_GetMicros()<MOVE_PERIOD_US) continue;	
		Timer_Switch(turn_on);							

		/* 1m 标定用:任意一路压上线就立刻停,后面交给循迹接管。
		   比赛流程里不用这个函数,所以不影响原先的定距直行语义。 */
		if (Sensor_GetErrorInformation().OnLineNum>0) break;

		Data=MPU_6050_GetTurnNeedData();
		if (Data.IsNew==New){
			MPU_6050_YawAngle_Update(&Data);			
			mpu_lost=0;
		}else if (++mpu_lost>MOVE_MPU_MAX){				
			break;
		}

		yaw=MPU_6050_GetYaw();

		/* Motor_GetMileage() 倒车时给的是负数,统一换算成"已经走了多少" */
		mileage=Motor_GetMileage();
		if (dir<0) mileage=-mileage;

		/* 防卡死:编码器连续 MOVE_STALL_MAX 拍没有变化,判定堵转,停轮退出(无状态返回) */
		if (mileage==mileage_last){
			if (++stall>MOVE_STALL_MAX) break;
		}else{
			stall=0;
			mileage_last=mileage;
		}

		remain=mag-mileage;
		if (remain<=0.0f) break;						//走够了
		if (yaw>MOVE_YAW_LIMIT||yaw<-MOVE_YAW_LIMIT) break;

		base=(remain<MOVE_SLOW_ZONE_CM)? MOVE_SLOW_PWM:MOVE_BASE_PWM;
		base=base*dir;

		u=MOVE_KP*(0.0f-yaw)-MOVE_KD*((float)Data.GyroZ/MPU_6050_GYRO_SENS);
		if (u>MOVE_STEER_MAX) u=MOVE_STEER_MAX;
		if (u<-MOVE_STEER_MAX) u=-MOVE_STEER_MAX;
		steer=(int16_t)u;

		/* 倒车时左右轮的修正量必须反号:同样的左右轮速差,倒着走产生的
		   偏航方向和正着走是相反的,不反号会越修越偏。 */
		Motor_SetSpeed(base+dir*steer,base-dir*steer);
	}
	Motor_SetSpeed(0,0);
}
