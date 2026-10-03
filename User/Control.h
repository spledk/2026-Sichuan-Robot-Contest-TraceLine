#ifndef Control_H
#define Control_H

#include <stdint.h>

//Car_TraceLine 的返回值,给上位函数判断用
enum Trace_Result{
	Trace_LostLine=0,	//脱线:盲走超时也没找回线
	Trace_OK=1,			//预留:正常终止条件(到达终点之类)
	Trace_AllOn=2		//四路全亮:路口/异常
};

/* ---------------- 场地拓扑 ----------------
   6 个 A4 方框,每个由一条 80cm 引导线接到中心交点(十字路口):
     0号=停车启动区(下方正中)  1号=左   2号=左上
     3号=上(0号正前方)        4号=右上  5号=右
   角度约定:以"车停在0号、车头朝向中心交点"为 0°,左转为正。 */
#define DEST_START		0
#define DEST_MAX		5

void Devices_Init(void);
void Interact(void);
void Car_TurnTo(float target_degree);
uint8_t Car_TraceLine(void);
void Car_MoveForward(float distance_cm);	//正=前进 负=倒车,单位 cm

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
