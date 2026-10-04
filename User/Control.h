#ifndef Control_H
#define Control_H

#include <stdint.h>

//Car_Act 的工作模式
enum Act_Mode{
	Act_All4=0,			//循迹,遇到四路全亮返回
	Act_All3,			//循迹,遇到三路全亮返回
	Act_Distance,		//定距循迹,里程到 distance 返回(不用陀螺仪)
	Act_Straight		//直线判断(循迹+陀螺仪),有效行程到 distance 返回
};

//Car_Act 的返回值:先列成功原因,再列异常;用 ACT_IS_OK 判断
enum Act_Result{
	Act_OK_All4=0,		//四路全亮
	Act_OK_All3,		//三路全亮
	Act_OK_Dist,		//定距走完
	Act_OK_Straight,	//直线判断走完
	Act_ErrLost,		//脱线
	Act_ErrStall,		//卡死
	Act_ErrMPU			//陀螺仪失灵
};
#define ACT_IS_OK(r)	((r)<Act_ErrLost)

/* ---------------- 场地拓扑 ----------------
   6 个 A4 方框,每个由一条 80cm 引导线接到中心交点(十字路口):
     0号=停车启动区(下方正中)  1号=左   2号=左上
     3号=上(0号正前方)        4号=右上  5号=右
   角度约定:以"车停在0号、车头朝向中心交点"为 0°,左转为正。 */
#define DEST_START		0
#define DEST_MAX		5

void Devices_Init(void);
void Interact(void);
uint8_t Car_Act(enum Act_Mode mode,float distance);	//集成动作,详见 Control.c 顶部注释
void Car_MoveForward(float distance_cm);	//正=前进 负=倒车,单位 cm

/* ---------------- 比赛流程 ----------------
   阶段1计数 -> 连续圆环 -> 出环后找直角弯 -> 停车 */
void Competition_Run(void);

/* ---------------- 模式函数(Interact 里按 Key2 调用) ---------------- */
void Mode_BackForth(void);		//基本(1):0号 <-> 3号 折返
void Mode_GoDestination(void);	//基本(2):0号 -> 随机目的地 -> 0号
void Mode_A2B(void);			//发挥(1):0号 -> A -> 停2s -> B -> 停2s -> 0号
void Mode_OffLine(void);		//发挥(2):0号 -> 巡线到1号 -> 离线直行到停车点

/* ---------------- 目标选择 ----------------
   不再是独立的两页,而是在按下 K2 进入某个模式之后,由
   Interface_SelectDest() 现场选,选完才开始跑。 */
extern uint8_t Dest_Main;		//目的地 / A->B 里的 A,范围 1~5
extern uint8_t Dest_Alt;		//A->B 里的 B / 离线停车点

/* ---------------- 运行状态(给 OLED 和上位判断用) ---------------- */
extern volatile uint8_t  Run_State;		//0=空闲 1=模式运行中
extern volatile uint8_t  Run_Current;	//正在前往的目的地编号
extern volatile uint16_t Run_Laps;		//已完成的折返次数
extern volatile uint16_t Run_Ticks;		//本模式已跑过的 10ms 节拍数

#endif
