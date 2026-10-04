#ifndef Control_H
#define Control_H

#include <stdint.h>

/* ---------------- Car_Act:集成动作函数 ----------------
   把"循迹 / 定距 / 陀螺仪"集成到一个循环里,一次调用只做一件事,做到就返回,
   由比赛状态机(main.c)线性串联。实现和几何依据见 Control.c 顶部注释。 */

enum Act_Mode{
	Act_All4=0,			//循迹,遇到四路全亮返回(十字路口)
	Act_All3,			//循迹,遇到三路全亮返回(直角弯)
	Act_Distance,		//定距循迹:里程到 distance 返回(不用陀螺仪)
	Act_Straight		//直线检测:有效行程攒够 distance 后再遇到三路全亮返回
};

enum Act_Result{
	Act_OK_All4=0,		//四路全亮
	Act_OK_All3,		//三路全亮
	Act_OK_Dist,		//定距走完
	Act_OK_Straight,	//直线检测走完(攒够直线又遇到三路)
	Act_ErrLost,		//脱线:盲走超时也没找回线
	Act_ErrStall,		//卡死:编码器连续一段时间没变化
	Act_ErrMPU			//陀螺仪失灵(只在 Act_Straight 里判)
};
#define ACT_IS_OK(r)	((r)<Act_ErrLost)	//先成功码后异常码,一句话判断

void Devices_Init(void);
void Interact(void);								//翻页选模式,在可跑的页上按 K2 之后返回
uint8_t Car_Act(enum Act_Mode mode,float distance);
void Car_MoveForward(float distance_cm);			//正=前进 负=倒车,单位 cm

#endif
