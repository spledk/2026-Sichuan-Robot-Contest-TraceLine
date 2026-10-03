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
#include <math.h>

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
	Motor_SetSpeed(0,0);				//顺手清一次,保证上电时两轮是停住的
	Interface(Page);

	while (1){
		if (Key1_GetStatus()==Press){	//K1:顺序翻页,翻过最后一页回主页
			Page++;
			if (Page>Page5_Back2MainInterface) Page=Page0_MainInterface;
			Interface(Page);
		}
		if (Key2_GetStatus()==Press){	//K2:进这一页对应的流程
			switch (Page){
				case Page0_MainInterface:
					break;

				case Page1_BackForth:		//0号 <-> 3号,不用选目的地
					Mode_BackForth();
					Interface(Page);
					break;

				case Page2_GoDestination:	//先选 1 个目的地(1~5),再跑
					Dest_Main=Interface_SelectDest("2.GO & RETURN","Dest",
					                               Dest_Main,1,DEST_MAX,0xFF);
					Mode_GoDestination();
					Interface(Page);
					break;

				case Page3_A2B:				//连选 2 个:A(1~5),B(1~5 且 !=A)
					Dest_Main=Interface_SelectDest("3.A TO B","A",
					                               Dest_Main,1,DEST_MAX,0xFF);
					Dest_Alt =Interface_SelectDest("3.A TO B","B",
					                               Dest_Alt, 1,DEST_MAX,Dest_Main);
					Mode_A2B();
					Interface(Page);
					break;

				case Page4_OffLine:			//停车点题目规定不能是 1、5 号,只在 2~4 循环
					Dest_Alt=Interface_SelectDest("4.OFF LINE","Stop",
					                              Dest_Alt,2,4,0xFF);
					Mode_OffLine();
					Interface(Page);
					break;

				case Page5_Back2MainInterface:
					Page=Page0_MainInterface;
					Interface(Page);
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
#define MOVE_SLOW_ZONE_CM	10.0f	//还剩这么多 cm 就切到 MOVE_SLOW_PWM(为了停准)
#define MOVE_KP			8.0f	
#define MOVE_KD			3.0f	
#define MOVE_STEER_MAX	250		
#define MOVE_YAW_LIMIT	30.0f	//意外事件导致偏转较大直接放弃
#define MOVE_MPU_MAX	100		//陀螺失联

void Car_MoveForward(float distance_cm){
	float mag=MyABS(distance_cm);				//要走多少 cm(不看符号)
	float remain=0.0f;
	struct MPU_6050_TurnNeedData Data;
	float yaw=0.0f,u=0.0f,mileage=0.0f;
	uint16_t mpu_lost=0;
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
 *    3) Car_MoveForward(NAV_SENSOR_CENTER_CM) 把测试点挪到交点上
 *    4) Car_TurnTo(辐条夹角) 转到 to 那条辐条
 *    5) 爬出交点黑块,压到 to 那条线               Nav_CreepOut()
 *    6) Car_TraceLine() 循迹到 to 号方框的黑边框   -> Trace_AllOn
 *    7) Car_MoveForward(NAV_PARK_CM) 进方框到几何中心(末段自动减速)
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

/* ---------------- 停车标定(上车实测这下面两项,停车精度全靠它) ----------------
   ① NAV_SENSOR_CENTER_CM : 传感器排 -> 小车"唯一测试点"(你指定的那个中心点)
      的距离,单位 cm。拿尺子量传感器板前沿到测试点的投影距离。
   ② NAV_BOX_LEN_CM : 停车区方框里"顺着进车方向"的那条边长,单位 cm。
      A4 纸是 21 x 29.7;六个方框进车方向都是走长边,量出来应该是 29.7。
   派生量:
      NAV_SPOKE_CM  = 方框黑边框 <-> 中心交点 = 引导线长度 = 80
      NAV_PARK_CM   = 传感器压到方框边框后还要往前开多少,测试点才落到方框
                      几何中心 = ① + ②/2
      NAV_CENTER_CM = 方框几何中心 <-> 中心交点 = ③ + ②/2
   注意 Car_TraceLine 停的时候,压住黑块的是"传感器排",而测试点在它后面
   ①那么远,所以每次都要再往前补一段,否则转向是绕传感器转的,会横着偏。 */
#define NAV_SENSOR_CENTER_CM	8.0f	//← 待实测:测试点(中心)到传感器排
#define NAV_BOX_LEN_CM			29.7f	//← 待实测:方框顺进车方向的边长
#define NAV_SPOKE_CM			80.0f	//引导线长度(实测)
#define NAV_PARK_CM				(NAV_SENSOR_CENTER_CM+NAV_BOX_LEN_CM*0.5f)
#define NAV_CENTER_CM			(NAV_SPOKE_CM+NAV_BOX_LEN_CM*0.5f)

/* ---------------- 离线直行的几何(发挥2 用) ----------------
   六个方框的几何中心,都在以中心交点为圆心、半径 NAV_CENTER_CM 的圆上,
   方位角就是上面那张 Spoke_Angle 表(0° = 车停在0号时车头方向朝上,左转为正)。
   单位方向向量 dir(θ) = (-sinθ, cosθ):
        θ=  0 -> ( 0.0000, 1.0000) 朝上(3号)
        θ= 45 -> (-0.7071, 0.7071) 左上(2号)
        θ= 90 -> (-1.0000, 0.0000) 朝左(1号)
        θ=180 -> ( 0.0000,-1.0000) 朝下(0号)
        θ=-45 -> ( 0.7071, 0.7071) 右上(4号)
        θ=-90 -> ( 1.0000, 0.0000) 朝右(5号)
   六个角度全是 45° 的整数倍,直接查表,不用 sin/cos。 */
static const float Spoke_DirX[6]={ 0.0f,-1.0f,-0.707107f, 0.0f, 0.707107f, 1.0f};
static const float Spoke_DirY[6]={-1.0f, 0.0f, 0.707107f, 1.0f, 0.707107f, 0.0f};

/* from 号方框几何中心 -> to 号方框几何中心 这条直线的长度(cm) */
static float Nav_DirectDist(uint8_t from,uint8_t to){
	float dx=NAV_CENTER_CM*(Spoke_DirX[to]-Spoke_DirX[from]);
	float dy=NAV_CENTER_CM*(Spoke_DirY[to]-Spoke_DirY[from]);
	return sqrtf(dx*dx+dy*dy);
}

/* 这条直线的绝对航向(度,和 yaw 同一套符号:左转为正)。
   两点都在同一个圆上,弦的方向 = 两端方位角的平均 ± 90°,取 +90 还是 -90
   看 to 在 from 的哪一侧(θ_to > θ_from 就 +90)。结果全是 22.5° 的整数倍,
   比拖进 atan2 干净:1->2 = -22.5°,1->3 = -45°,1->4 = -67.5°。 */
static float Nav_DirectHeading(uint8_t from,uint8_t to){
	float m=(Spoke_Angle[from]+Spoke_Angle[to])*0.5f;
	if (Spoke_Angle[to]>Spoke_Angle[from]) return m+90.0f;
	return m-90.0f;
}

/* ---------------- 折返返程的两种走法 ----------------
   0 = 到 3号方框中心后原地转 180°,再循迹走回来(稳,但每趟多两次转向)
   1 = 不转向,直接用定距直行倒车 2*NAV_CENTER_CM 回 0号方框中心。
       快得多,但返程全靠陀螺+编码器推算,横着偏了没东西纠正,一旦偏出
       2.5cm 线宽就直接判失败。先用 0 跑通,有把握了再切 1。 */
#define NAV_BACKFORTH_REVERSE	0

#define NAV_BACKFORTH_LAPS	3		//折返跑这么多趟就自己停(中途想停就把车抬起来 2~3s)

enum Nav_Result{
	Nav_OK=0,			//正常开到目标方框
	Nav_ErrLost=1,		//Car_TraceLine 没返回 Trace_AllOn(脱线了)
	Nav_ErrNoLine=2,	//爬行超距还没压到引导线
	Nav_ErrMPU=3		//陀螺失联
};

static char *Nav_Msg[4]={
	"OK","ERR LOST LINE","ERR NO LINE","ERR MPU LOST"
};

/* ---------------- 状态量 ---------------- */
uint8_t Dest_Main=3;				//默认去 3号(折返那条)
uint8_t Dest_Alt=5;					//A->B 里的 B / 离线停车点
volatile uint8_t  Run_State=0;
volatile uint8_t  Run_Current=0;
volatile uint16_t Run_Laps=0;
volatile uint16_t Run_Ticks=0;

/* ---------------- OLED 界面全部在 Interface.c,这里只留结果字符串 ---------------- */

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

	/* 3) 再往前 NAV_SENSOR_CENTER_CM,把小车的测试点挪到交点上。
	      Car_TraceLine 停的时候压住交点的是"传感器排",测试点还在它后面
	      ①那么远。先把这一段补上,下一步原地转向才是绕交点转的,转完车头
	      正好落在目标辐条上;不补的话每转一个角度就会横着偏出去
	      NAV_SENSOR_CENTER_CM*sin(转角),转90度能偏出十几厘米。 */
	Car_MoveForward(NAV_SENSOR_CENTER_CM);

	/* 4) 在交点上转到 to 号那条辐条(0号<->3号时夹角是 0,不用转) */
	turn=Spoke_Turn(from,to)*NAV_LEFT_SIGN;
	if (turn>NAV_TURN_SKIP||turn<-NAV_TURN_SKIP){
		Car_TurnTo(turn);
	}

	/* 5) 爬出交点黑块,压到 to 号那条线 */
	r=Nav_CreepOut();
	if (r!=Nav_OK) return r;

	/* 6) 循迹到 to 号方框的黑边框 */
	r=Car_TraceLine();
	if (r!=Trace_AllOn) return Nav_ErrLost;

	/* 7) 再往前 NAV_PARK_CM,测试点落到方框几何中心(最后 10cm 自动减速) */
	Car_MoveForward(NAV_PARK_CM);
	return Nav_OK;
}

/* ---------------- 上位:四种比赛模式 ---------------- */

/* 基本(1) 折返:0号 <-> 3号,30s 内折返次数越多越好
   0号<->3号 是一条笔直的 160cm 线,所以交点上不转向,只是路过。 */
void Mode_BackForth(void){
	uint8_t r=Nav_OK;

	Run_Laps=0; Run_Ticks=0; Run_State=1;
	Motor_Switch(turn_on);

	while (1){
		Run_Current=3;
		Interface_Run("GO",3);

		r=Nav_ParkAt(DEST_START,3);				//0号 -> 3号
		if (r!=Nav_OK) break;

#if NAV_BACKFORTH_REVERSE
		/* 方案2:不转向,直接倒车回 0号方框中心。
		   车头一直朝 3号,所以下一趟直接再往前开一次就行,不用再掉头。 */
		Run_Current=DEST_START;
		Interface_Run("REV",DEST_START);
		Car_MoveForward(-2.0f*NAV_CENTER_CM);
#else
		/* 方案1:原地掉头,再循迹走回 0号 */
		Car_TurnTo(180.0f);
		Run_Current=DEST_START;
		Interface_Run("BACK",DEST_START);
		r=Nav_ParkAt(3,DEST_START);				//3号 -> 0号
		if (r!=Nav_OK) break;

		Car_TurnTo(180.0f);						//再掉头,准备下一趟
#endif

		Run_Laps++;								//回到停车启动区 = 完成一次折返
		if (Run_Laps>=NAV_BACKFORTH_LAPS) break;
	}

	Motor_SetSpeed(0,0);
	Run_State=0;
	Interface_Finish(Nav_Msg[r<4?r:0]);
}

/* 基本(2) 从停车启动区出发,巡线到 Dest_Main,再巡线回来 */
void Mode_GoDestination(void){
	uint8_t r,d=Dest_Main;

	if (d<1||d>DEST_MAX) d=3;
	Run_Laps=0; Run_Ticks=0; Run_State=1;
	Motor_Switch(turn_on);

	Run_Current=d;
	Interface_Run("GO",d);
	r=Nav_ParkAt(DEST_START,d);

	if (r==Nav_OK){
		Delay_ms(500);							//停一下
		Car_TurnTo(180.0f);						//掉头往回开
		Run_Current=DEST_START;
		Interface_Run("BACK",DEST_START);
		r=Nav_ParkAt(d,DEST_START);
	}

	Motor_SetSpeed(0,0);
	Run_State=0;
	Interface_Finish(Nav_Msg[r<4?r:0]);
}

/* 发挥(1) 0号 -> A(Dest_Main) -> 停2s -> B(Dest_Alt) -> 停2s -> 0号
   本质就是"指定停车"这一个原语调三次,每次中间掉个头。 */
void Mode_A2B(void){
	uint8_t r,a=Dest_Main,b=Dest_Alt;

	if (a<1||a>DEST_MAX) a=3;
	if (b<1||b>DEST_MAX) b=4;
	if (b==a) b=(a==DEST_MAX)?4:(uint8_t)(a+1);	//A、B 不能是同一个

	Run_Laps=0; Run_Ticks=0; Run_State=1;
	Motor_Switch(turn_on);

	Run_Current=a;
	Interface_Run("GO A",a);
	r=Nav_ParkAt(DEST_START,a);

	if (r==Nav_OK){
		Delay_s(2);								//停车 2s
		Car_TurnTo(180.0f);

		Run_Current=b;
		Interface_Run("GO B",b);
		r=Nav_ParkAt(a,b);

		if (r==Nav_OK){
			Delay_s(2);							//停车 2s
			Car_TurnTo(180.0f);

			Run_Current=DEST_START;
			Interface_Run("BACK",DEST_START);
			r=Nav_ParkAt(b,DEST_START);
		}
	}

	Motor_SetSpeed(0,0);
	Run_State=0;
	Interface_Finish(Nav_Msg[r<4?r:0]);
}

/* 发挥(2) 巡线到 1号,之后不经巡线、一条直线直接开到停车点 Dest_Alt。
   题目规定停车点不能是 1、5 号,所以选择页只在 2~4 里循环。
   离线段 = "1号方框几何中心 -> 停车点方框几何中心" 的那条弦,不经过中心交点:
   两点都在半径 NAV_CENTER_CM 的圆上,距离和航向用上面的几何直接算。 */
void Mode_OffLine(void){
	uint8_t r,s=Dest_Alt;
	float dist,turn;

	if (s<2||s>4) s=3;						//兜底:选择页已经保证是 2~4
	Run_Laps=0; Run_Ticks=0; Run_State=1;
	Motor_Switch(turn_on);

	Run_Current=1;
	Interface_Run("GO",1);
	r=Nav_ParkAt(DEST_START,1);				//先循迹到 1号方框几何中心

	if (r!=Nav_OK){
		Motor_SetSpeed(0,0);
		Run_State=0;
		Interface_Finish(Nav_Msg[r<4?r:0]);
		return;
	}

	Delay_ms(500);							//在 1号稳一下再定位

	/* 此刻车停在 1号方框几何中心,车头朝 1号辐条向外的方向
	   (Car_TraceLine 是沿辐条从交点往外走进方框的,所以航向 = Spoke_Angle[1] = 90°)。
	   ① 先原地转 180° 掉头面向场地;
	   ② 再转到那条弦的航向。Car_TurnTo 的量是"从当前朝向算起的相对角",
	      掉头后朝向 = 90-180 = -90°,所以要转的量
	        = 弦航向 - (-90°) = 弦航向 + 90°
	        = (Spoke_Angle[1]+Spoke_Angle[s]) * 0.5f
	      代进去就是 2号 +67.5° / 3号 +45° / 4号 +22.5°,全是右转。 */
	Run_Current=s;
	Interface_Run("OFFLINE",s);

	Car_TurnTo(180.0f);						//① 掉头,面向场地
	turn=Nav_DirectHeading(1,s)+90.0f;		//② 再转到弦的航向
	Car_TurnTo(turn);

	dist=Nav_DirectDist(1,s);				//弦长:2号 72.6 / 3号 134.1 / 4号 175.3 cm
	Car_MoveForward(dist);					//一条直线开过去,全程不碰传感器

	Motor_SetSpeed(0,0);
	Run_State=0;
	Interface_Finish(Nav_Msg[r<4?r:0]);
}
