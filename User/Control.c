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

void Devices_Init(void){
	Timer_Init();
	OLED_Init();
	Motor_Init();
	Sensor_Init();
	MPU_6050_Init();	
}

void Interact(void){
	enum Interface_Page Page=Page0_MainInterface;
	Motor_Switch(turn_on);				//TB6612 STBY 拉高,否则电机永远不转
	Interface(Page);
	while (1){
		if (Key1_GetStatus()==Press){
			Page++;
			if (Page==Page7_Back2MainInterface) Page=Page0_MainInterface;
			Interface(Page);
		}
		if (Key2_GetStatus()==Press){
			switch (Page){
				case Page0_MainInterface:
					break;
				case Page1_BackForth:
					Mode_BackForth();
					Interface(Page);
					break;
				case Page2_GoDestination:
					Mode_GoDestination();
					Interface(Page);
					break;
				case Page3_A2B:
					Mode_A2B();
					Interface(Page);
					break;
				case Page4_OffLine:
					Mode_OffLine();
					Interface(Page);
					break;
				case Page5_ShowDestination:
					Dest_Main_Next();
					Interface(Page);
					break;
				case Page6_Config:
					Dest_Alt_Next();
					Interface(Page);
					break;
				case Page7_Back2MainInterface:
					break;
			}
		}
	}
}

//**原地转向**//
#define TURN_PERIOD_US		10000	//控制周期 10ms
#define TURN_KP				8.0f	
#define TURN_KD				3.0f	
#define TURN_DEADBAND		1.5f	//允许范围	
#define TURN_MIN_PWM		260		
#define TURN_SAMPLE_MAX		300		//3s超时
#define TURN_MPU_MAX		100		//陀螺失联

#define MyABS(x)				((x)<0 ? -(x) : (x))

void Car_TurnTo(float target_degree){
	struct MPU_6050_TurnNeedData Data;
	float err=0.0f,rate=0.0f,u=0.0f;
	uint16_t samples=0,mpu_lost=0;
	int16_t pwm=0;
					
	Motor_SetSpeed(0,0);		//默认电机开关已使能
	Delay_ms(100);								

	MPU_6050_GetTurnNeedData();					
	MPU_6050_SetYaw(0.0f);						
	Timer_Switch(turn_on);						

	while (1){
		if (Timer_GetMicros()<TURN_PERIOD_US) continue;	
		Timer_Switch(turn_on);							

		Data=MPU_6050_GetTurnNeedData();
		if (Data.IsNew==New){
			MPU_6050_YawAngle_Update(&Data);			
			mpu_lost=0;
		}else if (++mpu_lost>TURN_MPU_MAX){				
			break;
		}
		if (++samples>TURN_SAMPLE_MAX) break;			

		err=target_degree-MPU_6050_GetYaw();		
		if (MyABS(err)<=TURN_DEADBAND) break;		

		rate=(float)Data.GyroZ/MPU_6050_GYRO_SENS;	
		u=TURN_KP*err-TURN_KD*rate;					

		if (u>0.0f){								
			if (u<TURN_MIN_PWM) u=TURN_MIN_PWM;
		}else{
			if (u>-TURN_MIN_PWM) u=-TURN_MIN_PWM;
		}
		pwm=(int16_t)u;								
		Motor_SetSpeed(pwm,-pwm);					
	}
	Motor_SetSpeed(0,0);						
}

//**自动循迹**//
#define TRACE_PERIOD_US			10000	//控制周期 10ms
#define TRACE_KP				150.0f	
#define TRACE_KD				50.0f	

#define TRACE_BASE_PWM			350	
#define TRACE_STEER_MAX			400.0f	//转向限幅

#define TRACE_BLIND_ERROR		5.0f	//脱线打死
#define TRACE_BLIND_MAX			200		//脱线2s退出
#define TRACE_ALLON_CONFIRM		3		//4路识别判定 0ms

uint8_t Car_TraceLine(void){
	ErrorInformation Info;
	float error=0.0f,error_last=0.0f,steer=0.0f;
	int8_t dir=1;						//1=右 -1=左
	uint8_t result=Trace_OK;
	uint8_t allon=0;					//4路识别计时
	uint16_t blind=0;
	int16_t left=0,right=0;

	Motor_SetSpeed(0,0);
	Timer_Switch(turn_on);				

	while (1){
		if (Timer_GetMicros()<TRACE_PERIOD_US) continue;	
		Timer_Switch(turn_on);								

		Info=Sensor_GetErrorInformation();

		if (Info.OnLineNum==4){					//四路全亮:路口/异常
			if (++allon>=TRACE_ALLON_CONFIRM){ result=Trace_AllOn; break; }
			error=0.0f;							//确认期间直行
			error_last=0.0f;
		}else if (Info.OnLineNum==0){			//脱线打死
			allon=0;
			if (++blind>TRACE_BLIND_MAX){ result=Trace_LostLine; break; }
			error=dir*TRACE_BLIND_ERROR;
		}else{									//正常压线
			allon=0;
			blind=0;
			error=Info.error;
			if (error>0.1f) dir=1;				//直行不动dir
			else if (error<-0.1f) dir=-1;
		}

		steer=TRACE_KP*error+TRACE_KD*(error-error_last);
		error_last=error;

		if (steer>TRACE_STEER_MAX) steer=TRACE_STEER_MAX;
		if (steer<-TRACE_STEER_MAX) steer=-TRACE_STEER_MAX;

		left=TRACE_BASE_PWM+(int16_t)steer;
		right=TRACE_BASE_PWM-(int16_t)steer;

		Motor_SetSpeed(left,right);
	}
	Motor_SetSpeed(0,0);
	return result;
}

//**定距直行**//
#define MOVE_PERIOD_US	10000	//控制周期 10ms
#define MOVE_BASE_PWM	350		
#define MOVE_SLOW_PWM	200		//提前减速
#define MOVE_SLOW_RATIO	0.1f	//剩余里程少于总距离的10%就开始减速
#define MOVE_KP			8.0f	
#define MOVE_KD			3.0f	
#define MOVE_STEER_MAX	250		
#define MOVE_YAW_LIMIT	30.0f	//意外事件导致偏转较大直接放弃
#define MOVE_MPU_MAX	100		//陀螺失联

void Car_MoveForward(int16_t distance_cm){
	float slow_zone=(float)distance_cm*MOVE_SLOW_RATIO;	
	struct MPU_6050_TurnNeedData Data;
	float yaw=0.0f,u=0.0f,mileage=0.0f;
	uint16_t mpu_lost=0;
	int16_t steer=0,base=MOVE_BASE_PWM;

	Motor_SetSpeed(0,0);
	Motor_MileageCountSwitch(turn_on);		
	MPU_6050_GetTurnNeedData();				
	MPU_6050_SetYaw(0.0f);					
	Timer_Switch(turn_on);					

	while (1){
		if (Timer_GetMicros()<MOVE_PERIOD_US) continue;	
		Timer_Switch(turn_on);							

		Data=MPU_6050_GetTurnNeedData();
		if (Data.IsNew==New){
			MPU_6050_YawAngle_Update(&Data);			
			mpu_lost=0;
		}else if (++mpu_lost>MOVE_MPU_MAX){				
			break;
		}

		yaw=MPU_6050_GetYaw();

		mileage=Motor_GetMileage();
		if (mileage>=(float)distance_cm) break;
		if (yaw>MOVE_YAW_LIMIT||yaw<-MOVE_YAW_LIMIT) break;

		base=((float)distance_cm-mileage<slow_zone)? MOVE_SLOW_PWM:MOVE_BASE_PWM;

		u=MOVE_KP*(0.0f-yaw)-MOVE_KD*((float)Data.GyroZ/MPU_6050_GYRO_SENS);
		if (u>MOVE_STEER_MAX) u=MOVE_STEER_MAX;
		if (u<-MOVE_STEER_MAX) u=-MOVE_STEER_MAX;
		steer=(int16_t)u;

		Motor_SetSpeed(base+steer,base-steer);
	}
	Motor_SetSpeed(0,0);
}

/* ==================================================================
 *                        比赛流程部分
 *  跟上面的"基本模块"(转向/循迹/直行)分开,这里只负责"什么时候按顺序
 *  调用哪个模块",以及场地路由。
 *
 *  场地(由题目图1量测):
 *    6 个 A4 方框,各由一条 80cm 黑色引导线接到中心的十字交点。
 *      0号=停车启动区(下方正中)   1号=左    2号=左上
 *      3号=上(0号正前方)         4号=右上   5号=右
 *    0号 <-> 3号 是一条笔直的 160cm 线,折返不用转向。
 *
 *  一次"从 from 走到 to"的套路(每段都朝前开,不倒车):
 *    循迹到中心交点 -> 原地转到 to 那条辐条 -> 循迹到 to 方框 -> 再直行 15cm
 * ================================================================== */

/* ---------------- 转向标定(上车前必须验一次) ----------------
   1) 把车架空,调 Car_TurnTo(90.0f):
        - 车头稳稳停在 90° 附近          -> 闭环方向正确,继续第 2 步
        - 车头一路加速转不停 / 抖到 3s 超时 -> Car_TurnTo 里
          Motor_SetSpeed(pwm,-pwm);  要改成  Motor_SetSpeed(-pwm,pwm);
   2) 修好之后再调 Car_TurnTo(90.0f),看车头往哪边转:
        - 左转 -> NAV_LEFT_SIGN 保持 +1.0f
        - 右转 -> NAV_LEFT_SIGN 改成 -1.0f
   两个都对了,下面 6 条辐条的走向才对得上。 */
#define NAV_LEFT_SIGN		(+1.0f)

/* 辐条方位角(度),下标就是方框编号 */
static const float Spoke_Angle[6]={
	180.0f,		/* 0号 停车启动区 */
	 90.0f,		/* 1号 左   */
	 45.0f,		/* 2号 左上 */
	  0.0f,		/* 3号 上   */
	-45.0f,		/* 4号 右上 */
	-90.0f		/* 5号 右   */
};

/* 从 from 号走到中心交点后,要转到 to 号那条辐条所需转过的角(左正右负) */
static float Spoke_Turn(uint8_t from,uint8_t to){
	float a=Spoke_Angle[to]-Spoke_Angle[from]-180.0f;
	while (a> 180.0f) a-=360.0f;
	while (a<=-180.0f) a+=360.0f;
	return a;
}

/* ---------------- 导航参数 ---------------- */
#define NAV_PERIOD_US		10000	//控制周期 10ms
#define NAV_KP				150.0f	//和 TRACE_KP 同标定
#define NAV_KD				50.0f
#define NAV_BASE_PWM		350
#define NAV_STEER_MAX		400.0f
#define NAV_BLIND_ERROR		5.0f	//真脱线时按上次偏差方向满舵找回

#define NAV_HOLD_KP			8.0f	//出框/过黑块时的航向锁定(同 MOVE_KP)
#define NAV_HOLD_KD			3.0f
#define NAV_HOLD_MAX		250.0f
#define NAV_YAW_LIMIT		30.0f	//航向锁不住就放弃
#define NAV_SLOW_PWM		200
#define NAV_SLOW_ZONE		10.0f

#define NAV_CENTER_CM		95.0f	//方框几何中心 <-> 中心交点
#define NAV_BOX_IN_CM		15.0f	//方框黑边框 -> 方框几何中心(A4 短边/2≈14.85)

#define NAV_ALLON_CONFIRM	2		//连续 N 拍四路全亮才算"黑块"
#define NAV_JUNCTION_MIN_CM	45.0f	//里程超过它才可能是中心交点(排除起步压方框黑边框)
#define NAV_ACQUIRE_MAX_CM	35.0f	//出框后走这么远还没找到线 = 异常
#define NAV_LOST_MAX_TICK	40		//循迹中脱线 400ms = 异常
#define NAV_LEG_MAX_CM		220.0f	//单段行程上限保护
#define NAV_MPU_MAX			100		//连续 1s 没有新陀螺样本 = 异常

#define NAV_BACKFORTH_TICK	2800	//折返模式跑满约 28s 自己停(每个节拍 10ms)

enum Nav_Result{
	Nav_OK=0,			//线走完了,停在目标方框的黑边框上
	Nav_Junction=1,		//停在中心交点
	Nav_ErrLost=2,		//脱线找不回
	Nav_ErrNoLine=3,	//出框后一直找不到引导线
	Nav_ErrMPU=4,		//陀螺失联
	Nav_ErrFar=5,		//走太远还没到
	Nav_Abort=6			//用户按键中止
};

static char *Nav_Msg[7]={
	"OK","AT JUNCTION","ERR LOST LINE","ERR NO LINE",
	"ERR MPU LOST","ERR TOO FAR","ABORTED"
};

/* ---------------- 状态量 ---------------- */
uint8_t Dest_Main=3;				//默认去 3号(折返那条)
uint8_t Dest_Alt=5;					//A->B 里的 B / 离线停车点
volatile uint8_t  Run_State=0;
volatile uint8_t  Run_Current=0;
volatile uint16_t Run_Laps=0;
volatile uint16_t Run_Ticks=0;

static uint8_t Nav_AllowAbort=0;	//只有折返模式中途允许按键中止(它要跑 30s)

void Dest_Main_Next(void){
	Dest_Main++;
	if (Dest_Main>DEST_MAX) Dest_Main=1;
}
void Dest_Alt_Next(void){
	Dest_Alt++;
	if (Dest_Alt>DEST_MAX) Dest_Alt=1;
}

/* 直接读按键引脚,不能用 Keyx_GetStatus():那个会阻塞等松手,会把控制环卡死 */
static uint8_t Nav_KeyDown(void){
	if (!Nav_AllowAbort) return 0;
	if (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)==Bit_SET) return 1;
	if (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)==Bit_SET) return 1;
	return 0;
}

/* ---------------- OLED 小工具(Line 1~4,Column 1~16) ---------------- */
static void Nav_ShowRun(char *Tag,uint8_t Dest){
	OLED_Clear();
	OLED_ShowString(1,1,"RUN ");
	OLED_ShowString(1,5,Tag);
	OLED_ShowString(2,1,"Dest: ");
	OLED_ShowNum(2,7,Dest,1);
	OLED_ShowString(3,1,"Laps: ");
	OLED_ShowNum(3,7,Run_Laps,3);
	OLED_ShowString(4,1,"K1/K2: STOP");
}

static void Nav_ShowResult(uint8_t r){
	OLED_Clear();
	OLED_ShowString(1,1,"FINISHED");
	OLED_ShowString(2,1,Nav_Msg[r<7?r:0]);
	OLED_ShowString(3,1,"Laps: ");
	OLED_ShowNum(3,7,Run_Laps,3);
	OLED_ShowString(4,1,"ESC: K1/K2");
}

/* ---------------- 直线定距(航向锁定,不看传感器) ----------------
   用来:进方框的最后 15cm、以及发挥(2)里"不经巡线"的两段 95cm */
static void Nav_Straight(float cm){
	struct MPU_6050_TurnNeedData Data;
	float yaw=0.0f,u=0.0f,mile=0.0f;
	uint16_t mpu_lost=0;
	int16_t base=NAV_BASE_PWM,steer=0;
	uint8_t slow=0;

	Motor_SetSpeed(0,0);
	Motor_MileageCountSwitch(turn_on);
	MPU_6050_GetTurnNeedData();
	MPU_6050_SetYaw(0.0f);
	Timer_Switch(turn_on);

	while (1){
		if (Timer_GetMicros()<NAV_PERIOD_US) continue;
		Timer_Switch(turn_on);
		Run_Ticks++;

		if (Nav_KeyDown()) break;

		Data=MPU_6050_GetTurnNeedData();
		if (Data.IsNew==New){ MPU_6050_YawAngle_Update(&Data); mpu_lost=0; }
		else if (++mpu_lost>NAV_MPU_MAX) break;

		yaw=MPU_6050_GetYaw();
		mile=Motor_GetMileage();
		if (mile<0.0f) mile=-mile;
		if (mile>=cm) break;
		if (yaw>NAV_YAW_LIMIT||yaw<-NAV_YAW_LIMIT) break;

		if (!slow&&(cm-mile)<NAV_SLOW_ZONE){ slow=1; base=NAV_SLOW_PWM; }

		u=NAV_HOLD_KP*(0.0f-yaw)-NAV_HOLD_KD*((float)Data.GyroZ/MPU_6050_GYRO_SENS);
		if (u> NAV_HOLD_MAX) u= NAV_HOLD_MAX;
		if (u<-NAV_HOLD_MAX) u=-NAV_HOLD_MAX;
		steer=(int16_t)u;

		Motor_SetSpeed(base+steer,base-steer);
	}
	Motor_SetSpeed(0,0);
}

/* ---------------- 循迹巡航(核心) ----------------
   stop_junc=1 : 从方框出发,走到中心交点就停(第一段)
   stop_junc=0 : 从交点出发/贯穿,一直走到引导线走完(线尽头就是方框黑边框)

   黑块(四路全亮)的两种含义靠"黑块后面接什么"区分:
     黑块 -> 又是引导线  = 中心交点,冲过去
     黑块 -> 白纸        = 方框黑边框,到站了
   这样完全不吃起始位置,不用管车停在方框里的哪个位置。 */
static uint8_t Nav_Cruise(uint8_t stop_junc){
	ErrorInformation Info;
	struct MPU_6050_TurnNeedData Data;
	float error=0.0f,error_last=0.0f,steer=0.0f,u=0.0f,yaw=0.0f,mile=0.0f;
	int8_t blind_dir=1;
	uint8_t allon=0,black=0,acquired=0,result=Nav_OK;
	uint16_t lost=0,mpu_lost=0;
	int16_t left=0,right=0;

	Motor_SetSpeed(0,0);
	Motor_MileageCountSwitch(turn_on);
	MPU_6050_GetTurnNeedData();
	MPU_6050_SetYaw(0.0f);
	Timer_Switch(turn_on);

	while (1){
		if (Timer_GetMicros()<NAV_PERIOD_US) continue;
		Timer_Switch(turn_on);
		Run_Ticks++;

		if (Nav_KeyDown()){ result=Nav_Abort; break; }

		Data=MPU_6050_GetTurnNeedData();
		if (Data.IsNew==New){ MPU_6050_YawAngle_Update(&Data); mpu_lost=0; }
		else if (++mpu_lost>NAV_MPU_MAX){ result=Nav_ErrMPU; break; }

		mile=Motor_GetMileage();
		if (mile<0.0f) mile=-mile;
		if (mile>NAV_LEG_MAX_CM){ result=Nav_ErrFar; break; }

		Info=Sensor_GetErrorInformation();

		if (!acquired){
			/* 出框阶段:方框里面是白纸,先锁航向直行把引导线找回来 */
			if (Info.OnLineNum>0&&Info.OnLineNum<4) acquired=1;
			else{
				if (mile>NAV_ACQUIRE_MAX_CM){ result=Nav_ErrNoLine; break; }
				yaw=MPU_6050_GetYaw();
				u=NAV_HOLD_KP*(0.0f-yaw)-NAV_HOLD_KD*((float)Data.GyroZ/MPU_6050_GYRO_SENS);
				if (u> NAV_HOLD_MAX) u= NAV_HOLD_MAX;
				if (u<-NAV_HOLD_MAX) u=-NAV_HOLD_MAX;
				Motor_SetSpeed(NAV_BASE_PWM+(int16_t)u,NAV_BASE_PWM-(int16_t)u);
				continue;
			}
		}

		if (Info.OnLineNum==4){
			/* 黑块:锁航向直行冲过去,并记下"刚压过黑块" */
			if (++allon>=NAV_ALLON_CONFIRM) black=1;
			error=0.0f; error_last=0.0f;
			if (stop_junc&&black&&mile>NAV_JUNCTION_MIN_CM){ result=Nav_Junction; break; }
			yaw=MPU_6050_GetYaw();
			u=NAV_HOLD_KP*(0.0f-yaw)-NAV_HOLD_KD*((float)Data.GyroZ/MPU_6050_GYRO_SENS);
			if (u> NAV_HOLD_MAX) u= NAV_HOLD_MAX;
			if (u<-NAV_HOLD_MAX) u=-NAV_HOLD_MAX;
			Motor_SetSpeed(NAV_BASE_PWM+(int16_t)u,NAV_BASE_PWM-(int16_t)u);
			continue;
		}

		if (Info.OnLineNum==0){
			allon=0;
			if (black){ result=Nav_OK; break; }		//黑块后面是白纸 => 到方框了
			if (++lost>NAV_LOST_MAX_TICK){ result=Nav_ErrLost; break; }
			error=(float)blind_dir*NAV_BLIND_ERROR;
		}else{
			allon=0; black=0; lost=0;
			error=Info.error;
			if (error> 0.1f) blind_dir= 1;
			if (error<-0.1f) blind_dir=-1;
		}

		steer=NAV_KP*error+NAV_KD*(error-error_last);
		error_last=error;
		if (steer> NAV_STEER_MAX) steer= NAV_STEER_MAX;
		if (steer<-NAV_STEER_MAX) steer=-NAV_STEER_MAX;

		left =NAV_BASE_PWM+(int16_t)steer;
		right=NAV_BASE_PWM-(int16_t)steer;
		Motor_SetSpeed(left,right);
	}

	Motor_SetSpeed(0,0);
	return result;
}

/* ---------------- 从 from 号方框走到 to 号方框(全朝前开) ---------------- */
static uint8_t Nav_GotoDest(uint8_t from,uint8_t to){
	uint8_t r;

	if (from==to) return Nav_OK;

	r=Nav_Cruise(1);						//方框 -> 中心交点
	if (r!=Nav_Junction) return r;

	Car_TurnTo(Spoke_Turn(from,to)*NAV_LEFT_SIGN);

	r=Nav_Cruise(0);						//交点 -> 目标方框黑边框
	if (r!=Nav_OK) return r;

	Nav_Straight(NAV_BOX_IN_CM);			//再往里 15cm,测试点落到方框几何中心
	return Nav_OK;
}

/* ---------------- 上位:四种比赛模式 ---------------- */

/* 基本(1) 折返:0号 <-> 3号,30s 内次数越多越好 */
void Mode_BackForth(void){
	uint8_t r=Nav_OK,to=3;

	Run_Laps=0; Run_Ticks=0; Run_State=1; Nav_AllowAbort=1;
	Motor_Switch(turn_on);

	while (1){
		Run_Current=to;
		Nav_ShowRun("GO",to);

		r=Nav_Cruise(0);					//一路朝前,穿过中心交点,直到线走完
		if (r!=Nav_OK) break;

		Nav_Straight(NAV_BOX_IN_CM);		//进入方框到几何中心
		if (to==DEST_START) Run_Laps++;		//又回到停车启动区 = 完成一次折返

		if (Run_Ticks>=NAV_BACKFORTH_TICK) break;

		Nav_ShowRun("TURN",to);
		Car_TurnTo(180.0f);					//原地掉头
		to=(to==3)?DEST_START:3;
	}

	Motor_SetSpeed(0,0);
	Nav_AllowAbort=0; Run_State=0;
	Nav_ShowResult(r);
}

/* 基本(2) 从停车启动区出发,巡线到 Dest_Main,再巡线回来 */
void Mode_GoDestination(void){
	uint8_t r,d=Dest_Main;

	if (d<1||d>DEST_MAX) d=3;
	Run_Ticks=0; Run_State=1; Nav_AllowAbort=1;
	Motor_Switch(turn_on);

	Run_Current=d;
	Nav_ShowRun("GO",d);
	r=Nav_GotoDest(DEST_START,d);

	if (r==Nav_OK){
		Delay_ms(500);						//停一下
		Car_TurnTo(180.0f);					//掉头
		Run_Current=DEST_START;
		Nav_ShowRun("BACK",DEST_START);
		r=Nav_GotoDest(d,DEST_START);
	}

	Motor_SetSpeed(0,0);
	Nav_AllowAbort=0; Run_State=0;
	Nav_ShowResult(r);
}

/* 发挥(1) 0号 -> A(Dest_Main) -> 停2s -> B(Dest_Alt) -> 停2s -> 0号 */
void Mode_A2B(void){
	uint8_t r,a=Dest_Main,b=Dest_Alt;

	if (a<1||a>DEST_MAX) a=3;
	if (b<1||b>DEST_MAX) b=4;
	if (b==a) b=(a==DEST_MAX)?4:(uint8_t)(a+1);		//A、B 不能是同一个

	Run_Ticks=0; Run_State=1; Nav_AllowAbort=1;
	Motor_Switch(turn_on);

	Run_Current=a;
	Nav_ShowRun("GO A",a);
	r=Nav_GotoDest(DEST_START,a);

	if (r==Nav_OK){
		Delay_s(2);							//停车 2s
		Car_TurnTo(180.0f);

		Run_Current=b;
		Nav_ShowRun("GO B",b);
		r=Nav_GotoDest(a,b);

		if (r==Nav_OK){
			Delay_s(2);						//停车 2s
			Car_TurnTo(180.0f);

			Run_Current=DEST_START;
			Nav_ShowRun("BACK",DEST_START);
			r=Nav_GotoDest(b,DEST_START);
		}
	}

	Motor_SetSpeed(0,0);
	Nav_AllowAbort=0; Run_State=0;
	Nav_ShowResult(r);
}

/* 发挥(2) 巡线到 1号,之后不经巡线直线开到停车点 Dest_Alt(题目规定不能是1、5号) */
void Mode_OffLine(void){
	uint8_t r,s=Dest_Alt;

	if (s<2||s>4) s=3;
	Run_Ticks=0; Run_State=1; Nav_AllowAbort=1;
	Motor_Switch(turn_on);

	Run_Current=1;
	Nav_ShowRun("GO",1);
	r=Nav_GotoDest(DEST_START,1);

	if (r==Nav_OK){
		Delay_ms(500);

		/* ==== 离线阶段:不再看传感器,只用陀螺航向 + 编码器里程 ==== */
		Car_TurnTo(180.0f);					//掉头,面向中心交点
		Run_Current=s;
		Nav_ShowRun("OFFLINE",s);
		Nav_Straight(NAV_CENTER_CM);		//直线开到中心交点
		Car_TurnTo(Spoke_Turn(1,s)*NAV_LEFT_SIGN);
		Nav_Straight(NAV_CENTER_CM);		//直线开到 s 号方框几何中心
	}

	Motor_SetSpeed(0,0);
	Nav_AllowAbort=0; Run_State=0;
	Nav_ShowResult(r);
}
