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
	Act_Straight,		//直线检测:有效行程攒够 distance 后再遇到三路/四路全亮返回
	Act_Calib			//1m标定:循迹+记里程,一脱线就用走过的里程反推 CountPerCM 并返回
};

enum Act_Result{
	Act_OK_All4=0,		//四路全亮
	Act_OK_All3,		//三路全亮
	Act_OK_Dist,		//定距走完
	Act_OK_Straight,	//直线检测走完(攒够直线又遇到三路/四路)
	Act_OK_Calib,		//1m标定走完(脱线),CountPerCM 已更新
	Act_ErrLost,		//脱线:盲走超时也没找回线
	Act_ErrStall,		//卡死:编码器连续一段时间没变化
	Act_ErrMPU			//陀螺仪失灵(只在 Act_Straight 里判)
};
#define ACT_IS_OK(r)	((r)<Act_ErrLost)	//先成功码后异常码,一句话判断

void Devices_Init(void);

/* ---------------- 交互界面 ---------------- */
/* Interact() 里跑完整的翻页+按键循环,只有两种返回:
     1 = 用户在"开始比赛"页按了 K2,main 应该去跑比赛状态机
     0 = 用户在配置页跑完了一个功能(按完确认),main 直接 continue 再进来
   所以 main 里的写法是:  if (Interact()==0) continue;            */
uint8_t Interact(void);

void Countdown_Show(uint8_t sec);			//sec 秒倒计时(画在 OLED 第4行),数完显示 GO!
void Wait_Key2(void);						//等 K2 确认
void Gyro_Calib(void);						//配置:陀螺仪零偏校准(3s 倒数 + 校准 + 等确认)
void Meter_Calib(void);						//配置:1m 里程标定(3s 倒数 + 直行上线 + 循迹测距 + 等确认)

/* main 的状态机显示:stage 阶段号,junc 已数到的四路全亮次数,
   res 停车原因码,stopped 0=正在跑 1=已停 */
void Run_Show(uint8_t stage,uint8_t junc,uint8_t res,uint8_t stopped);

uint8_t Car_Act(enum Act_Mode mode,float distance);
void Car_MoveForward(float distance_cm);			//正=前进 负=倒车,单位 cm。压上线(任一路亮)就返回

extern float Act_StraightValid;						//Act_Straight 当前累计的直线有效行程(cm),标定用

#endif
