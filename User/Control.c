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
 *  跟上面的"基本模块"(转向/循迹/直行)完全分开。这里只负责
 *  "什么时候按顺序调用哪个基本模块",以及场地路由。
 *
 *  三个基本模块一行不改,原样调用:
 *      Car_TraceLine()    一次性循迹:从压线处一路开,遇到四路全亮(黑块)
 *                         或脱线 2s 就返回,返回时把电机停掉
 *      Car_TurnTo(deg)    原地转到相对角(它内部先把 yaw 归零)
 *      Car_MoveForward(n) 锁航向直行 n cm(不看传感器)
 *
 *  场地(由题目图1量测 + 用户确认):
 *    6 个 A4 方框,各由一条 80cm 黑色引导线接到中心的十字交点。
 *      0号=停车启动区(下方正中)   1号=左    2号=左上
 *      3号=上(0号正前方)         4号=右上   5号=右
 *    0号 <-> 3号 是一条笔直的 160cm 线,折返不用转向。
 *
 *  一次"从 from 号走到 to 号"的固定流程(全程朝前开,不倒车):
 *    1) 从方框里的白纸爬出来,压到引导线          Nav_CreepOut()
 *    2) Car_TraceLine() 循迹到中心交点            -> Trace_AllOn
 *    3) Car_TurnTo(辐条夹角) 转到 to 那条辐条
 *    4) 爬出交点黑块,压到 to 那条线               Nav_CreepOut()
 *    5) Car_TraceLine() 循迹到 to 号方框的黑边框   -> Trace_AllOn
 *    6) Car_MoveForward(15) 进方框到几何中心
 *
 *  这里没有"数第几次退出"的状态机,因为 Car_TraceLine 的两次退出
 *  天然就被两个 Nav_CreepOut() 隔开了,顺序是定死的。
 *
 *  两个坑(都靠 Nav_CreepOut 绕开):
 *    a) 方框内部是白纸,车不在线上。直接调 Car_TraceLine() 第一拍
 *       就是 OnLineNum==0,走"脱线打死"分支(error=±5 -> steer=±750
 *       被限到 ±400 -> 左右轮 750/-50),车会原地猛拐。
 *    b) Car_TraceLine() 返回时车正停在黑块上。这时候直接再调它,
 *       它第一拍又是 OnLineNum==4,原地再报一次 Trace_AllOn,永远
 *       走不掉。所以每次退出之后都必须先爬出黑块。
 * ================================================================== */

/* ---------------- 转向标定(上车前必须验一次) ----------------
   1) 把车架空,调 Car_TurnTo(90.0f):
        - 车头稳稳停在 90° 附近            -> 闭环方向正确,继续第 2 步
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
#define NAV_PERIOD_US		10000	//控制周期 10ms(和基本模块一致)
#define NAV_CREEP_PWM		250		//爬行速度,别快
#define NAV_CREEP_MAX_CM	35.0f	//爬这么远还没压到线 = 异常(方框中心到边框约15cm)
#define NAV_HOLD_KP			8.0f	//爬行时的航向锁定(同 MOVE_KP/MOVE_KD)
#define NAV_HOLD_KD			3.0f
#define NAV_HOLD_MAX		250.0f
#define NAV_MPU_MAX			100		//连续 1s 没有新陀螺样本 = 异常
#define NAV_TURN_SKIP		1.0f	//辐条夹角小于它就不转向(0号<->3号时是 0)

#define NAV_CENTER_CM		95.0f	//方框几何中心 <-> 中心交点(发挥2离线用)
#define NAV_BOX_IN_CM		15.0f	//方框黑边框 -> 方框几何中心(A4 短边/2≈14.85)

#define NAV_BACKFORTH_LAPS	3		//折返跑这么多趟就自己停(也可随时按键中止)

enum Nav_Result{
	Nav_OK=0,			//正常开到目标方框
	Nav_ErrLost=1,		//Car_TraceLine 没返回 Trace_AllOn(脱线了)
	Nav_ErrNoLine=2,	//爬行超距还没压到引导线
	Nav_ErrMPU=3,		//陀螺失联
	Nav_Abort=4			//用户按键中止
};

static char *Nav_Msg[5]={
	"OK","ERR LOST LINE","ERR NO LINE","ERR MPU LOST","ABORTED"
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
	OLED_ShowString(2,1,Nav_Msg[r<5?r:0]);
	OLED_ShowString(3,1,"Laps: ");
	OLED_ShowNum(3,7,Run_Laps,3);
	OLED_ShowString(4,1,"ESC: K1/K2");
}

/* ---------------- 爬行:低速直行,直到传感器重新看到 1~3 路压线 ----------------
   两种情况都用它:
     - 从方框里的白纸爬出来(中间会经过方框的 1cm 黑边框,那是 4 路全亮,不算)
     - 从 Car_TraceLine 退出时停着的黑块上爬出来
   返回 Nav_OK 表示已经压到引导线上,车也停住了。 */
static uint8_t Nav_CreepOut(void){
	struct MPU_6050_TurnNeedData Data;
	ErrorInformation Info;
	float yaw=0.0f,u=0.0f,mile=0.0f;
	uint16_t mpu_lost=0;

	Motor_SetSpeed(0,0);
	Motor_MileageCountSwitch(turn_on);
	MPU_6050_GetTurnNeedData();
	MPU_6050_SetYaw(0.0f);
	Timer_Switch(turn_on);

	while (1){
		if (Timer_GetMicros()<NAV_PERIOD_US) continue;
		Timer_Switch(turn_on);
		Run_Ticks++;

		if (Nav_KeyDown()){ Motor_SetSpeed(0,0); return Nav_Abort; }

		Data=MPU_6050_GetTurnNeedData();
		if (Data.IsNew==New){ MPU_6050_YawAngle_Update(&Data); mpu_lost=0; }
		else if (++mpu_lost>NAV_MPU_MAX){ Motor_SetSpeed(0,0); return Nav_ErrMPU; }

		Info=Sensor_GetErrorInformation();
		if (Info.OnLineNum>0&&Info.OnLineNum<4){		//重新压到线了
			Motor_SetSpeed(0,0);
			return Nav_OK;
		}

		mile=Motor_GetMileage();
		if (mile<0.0f) mile=-mile;
		if (mile>NAV_CREEP_MAX_CM){						//爬太远了,线丢了
			Motor_SetSpeed(0,0);
			return Nav_ErrNoLine;
		}

		yaw=MPU_6050_GetYaw();							//锁航向直行
		u=NAV_HOLD_KP*(0.0f-yaw)-NAV_HOLD_KD*((float)Data.GyroZ/MPU_6050_GYRO_SENS);
		if (u> NAV_HOLD_MAX) u= NAV_HOLD_MAX;
		if (u<-NAV_HOLD_MAX) u=-NAV_HOLD_MAX;
		Motor_SetSpeed(NAV_CREEP_PWM+(int16_t)u,NAV_CREEP_PWM-(int16_t)u);
	}
}

/* ---------------- 核心原语:从 from 号方框开到 to 号方框 ----------------
   约定:调用前车头必须朝向"中心交点"那一侧。
        刚从某个方框到站时车头是背对交点的,所以模式函数要先
        Car_TurnTo(180.0f) 掉头,再调本函数。 */
static uint8_t Nav_ParkAt(uint8_t from,uint8_t to){
	uint8_t r;
	float turn;

	if (from==to) return Nav_OK;

	/* 1) 从方框里的白纸爬出来,压到引导线 */
	r=Nav_CreepOut();
	if (r!=Nav_OK) return r;

	/* 2) 循迹到中心交点:Car_TraceLine 遇到黑块就返回 */
	r=Car_TraceLine();
	if (r!=Trace_AllOn) return Nav_ErrLost;

	/* 3) 在交点上转到 to 号那条辐条(0号<->3号时夹角是 0,不用转) */
	turn=Spoke_Turn(from,to)*NAV_LEFT_SIGN;
	if (turn>NAV_TURN_SKIP||turn<-NAV_TURN_SKIP){
		Car_TurnTo(turn);
	}

	/* 4) 爬出交点黑块,压到 to 号那条线 */
	r=Nav_CreepOut();
	if (r!=Nav_OK) return r;

	/* 5) 循迹到 to 号方框的黑边框 */
	r=Car_TraceLine();
	if (r!=Trace_AllOn) return Nav_ErrLost;

	/* 6) 再往前 15cm,测试点落到方框几何中心 */
	Car_MoveForward((int16_t)NAV_BOX_IN_CM);
	return Nav_OK;
}

/* ---------------- 上位:四种比赛模式 ---------------- */

/* 基本(1) 折返:0号 <-> 3号,30s 内折返次数越多越好
   0号<->3号 是一条笔直的 160cm 线,所以交点上不转向,只是路过。 */
void Mode_BackForth(void){
	uint8_t r=Nav_OK,from=DEST_START,to=3,t;

	Run_Laps=0; Run_Ticks=0; Run_State=1; Nav_AllowAbort=1;
	Motor_Switch(turn_on);

	while (1){
		Run_Current=to;
		Nav_ShowRun("GO",to);

		r=Nav_ParkAt(from,to);					//一站
		if (r!=Nav_OK) break;

		if (to==DEST_START) Run_Laps++;			//回到停车启动区 = 完成一次折返
		if (Run_Laps>=NAV_BACKFORTH_LAPS) break;

		Car_TurnTo(180.0f);						//到站后原地掉头,下一趟往回开
		t=from; from=to; to=t;
	}

	Motor_SetSpeed(0,0);
	Nav_AllowAbort=0; Run_State=0;
	Nav_ShowResult(r);
}

/* 基本(2) 从停车启动区出发,巡线到 Dest_Main,再巡线回来 */
void Mode_GoDestination(void){
	uint8_t r,d=Dest_Main;

	if (d<1||d>DEST_MAX) d=3;
	Run_Laps=0; Run_Ticks=0; Run_State=1; Nav_AllowAbort=1;
	Motor_Switch(turn_on);

	Run_Current=d;
	Nav_ShowRun("GO",d);
	r=Nav_ParkAt(DEST_START,d);

	if (r==Nav_OK){
		Delay_ms(500);							//停一下
		Car_TurnTo(180.0f);						//掉头往回开
		Run_Current=DEST_START;
		Nav_ShowRun("BACK",DEST_START);
		r=Nav_ParkAt(d,DEST_START);
	}

	Motor_SetSpeed(0,0);
	Nav_AllowAbort=0; Run_State=0;
	Nav_ShowResult(r);
}

/* 发挥(1) 0号 -> A(Dest_Main) -> 停2s -> B(Dest_Alt) -> 停2s -> 0号
   本质就是"指定停车"这一个原语调三次,每次中间掉个头。 */
void Mode_A2B(void){
	uint8_t r,a=Dest_Main,b=Dest_Alt;

	if (a<1||a>DEST_MAX) a=3;
	if (b<1||b>DEST_MAX) b=4;
	if (b==a) b=(a==DEST_MAX)?4:(uint8_t)(a+1);	//A、B 不能是同一个

	Run_Laps=0; Run_Ticks=0; Run_State=1; Nav_AllowAbort=1;
	Motor_Switch(turn_on);

	Run_Current=a;
	Nav_ShowRun("GO A",a);
	r=Nav_ParkAt(DEST_START,a);

	if (r==Nav_OK){
		Delay_s(2);								//停车 2s
		Car_TurnTo(180.0f);

		Run_Current=b;
		Nav_ShowRun("GO B",b);
		r=Nav_ParkAt(a,b);

		if (r==Nav_OK){
			Delay_s(2);							//停车 2s
			Car_TurnTo(180.0f);

			Run_Current=DEST_START;
			Nav_ShowRun("BACK",DEST_START);
			r=Nav_ParkAt(b,DEST_START);
		}
	}

	Motor_SetSpeed(0,0);
	Nav_AllowAbort=0; Run_State=0;
	Nav_ShowResult(r);
}

/* 发挥(2) 巡线到 1号,之后不经巡线直线开到停车点 Dest_Alt
   题目规定停车点不能是 1、5 号,所以 s 夹到 2~4。
   离线段走"方框中心 -> 中心交点 -> 停车点方框中心"两段各 95cm,
   全程只用陀螺航向 + 编码器里程,不碰传感器。 */
void Mode_OffLine(void){
	uint8_t r,s=Dest_Alt;
	float turn;

	if (s<2||s>4) s=3;
	Run_Laps=0; Run_Ticks=0; Run_State=1; Nav_AllowAbort=1;
	Motor_Switch(turn_on);

	Run_Current=1;
	Nav_ShowRun("GO",1);
	r=Nav_ParkAt(DEST_START,1);

	if (r==Nav_OK){
		Delay_ms(500);
		Car_TurnTo(180.0f);						//掉头,面向中心交点

		Run_Current=s;
		Nav_ShowRun("OFFLINE",s);

		Car_MoveForward((int16_t)NAV_CENTER_CM);	//直线开到中心交点

		turn=Spoke_Turn(1,s)*NAV_LEFT_SIGN;		//在交点上转到停车点那条辐条
		if (turn>NAV_TURN_SKIP||turn<-NAV_TURN_SKIP){
			Car_TurnTo(turn);
		}

		Car_MoveForward((int16_t)NAV_CENTER_CM);	//直线开到停车点方框几何中心
	}

	Motor_SetSpeed(0,0);
	Nav_AllowAbort=0; Run_State=0;
	Nav_ShowResult(r);
}
