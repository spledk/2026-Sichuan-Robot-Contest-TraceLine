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

void Interact(void){
	enum Interface_Page Page=Page0_MainInterface;

	Motor_Switch(turn_on);				
	Motor_SetSpeed(0,0);				
	Interface(Page);

	while (1){
		if (Key1_GetStatus()==Press){	//K1:顺序翻页,翻过最后一页回主页
			Page++;
			if (Page>Pageend) Page=Page0_MainInterface;
			Interface(Page);
		}
		if (Key2_GetStatus()==Press){	//K2:进这一页对应的流程
			switch (Page){
				case Page0_MainInterface:
					break;
				case Pageend:
					return;				//选好了,回 main 去跑比赛状态机
			}
		}
	}
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
                  角度为新起点重新累计;有效行程攒够 distance 之后,再遇到三路全亮
                  就返回(本工程不需要直线检测四路,默认只认三路)。
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

uint8_t Car_Act(enum Act_Mode mode,float distance){
	ErrorInformation Info;
	struct MPU_6050_TurnNeedData Data;
	float error=0.0f,error_last=0.0f,steer=0.0f;
	float yaw=0.0f,yaw_ref=0.0f,valid=0.0f;
	float mileage=0.0f,mileage_last=0.0f,dm=0.0f;
	int8_t dir=1;						//1=右 -1=左
	uint8_t result=Act_OK_Dist;			//占位,循环里必被覆盖
	uint16_t blind=0,stall=0,mpu_lost=0,all3=0;
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
			if (valid>=distance&&Info.OnLineNum==3){ result=Act_OK_Straight; break; }
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
