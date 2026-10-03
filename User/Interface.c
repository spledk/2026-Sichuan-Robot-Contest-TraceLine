#include "stm32f10x.h"
#include "Interface.h"
#include "Control.h"
#include "Key.h"
#include "OLED.h"

/* OLED 是 128x64 8x16 字库:Line 1~4、Column 1~16,而且库里没有中文字模,
   所以界面全部用 ASCII,每行不超过 16 个字符。

   这个文件放的是**全部**界面:
     - 第一层选择界面的每一页(Page0~Page5)
     - 目的地选择页    Interface_SelectDest()
     - 运行页          Interface_Run()      <- 题目要求行驶中显示当前目的地
     - 结果页          Interface_Finish()
   Control.c 里的 Interact() 只负责翻页和分发,不画东西。 */

void Interface(enum Interface_Page Page){
	switch (Page){
		case Page0_MainInterface:
			OLED_Clear();
			OLED_ShowString(1,1,"== TRACE CAR ==");
			OLED_ShowString(2,1,"Dest: ");
			OLED_ShowNum(2,7,Dest_Main,1);
			OLED_ShowString(3,1,"Laps: ");
			OLED_ShowNum(3,7,Run_Laps,3);
			OLED_ShowString(4,1,"K1:Page  K2:Run");
			break;

		case Page1_BackForth:
			OLED_Clear();
			OLED_ShowString(1,1,"1.BACK & FORTH");
			OLED_ShowString(2,1,"0 <-> 3 (auto)");
			OLED_ShowString(3,1,"no setup need");
			OLED_ShowString(4,1,"K1:Page K2:Start");
			break;

		case Page2_GoDestination:
			OLED_Clear();
			OLED_ShowString(1,1,"2.GO & RETURN");
			OLED_ShowString(2,1,"0 -> dest -> 0");
			OLED_ShowString(3,1,"pick dest next");
			OLED_ShowString(4,1,"K1:Page K2:Start");
			break;

		case Page3_A2B:
			OLED_Clear();
			OLED_ShowString(1,1,"3.A TO B");
			OLED_ShowString(2,1,"0 ->A stop2s");
			OLED_ShowString(3,1,"->B stop2s ->0");
			OLED_ShowString(4,1,"K1:Page K2:Start");
			break;

		case Page4_OffLine:
			OLED_Clear();
			OLED_ShowString(1,1,"4.OFF LINE");
			OLED_ShowString(2,1,"trace to 1 then");
			OLED_ShowString(3,1,"go straight");
			OLED_ShowString(4,1,"K1:Page K2:Start");
			break;

		case Page5_Back2MainInterface:
			OLED_Clear();
			OLED_ShowString(1,1,"5.BACK");
			OLED_ShowString(2,1,"K1:Main");
			break;
	}
}

/* ---------------- 目的地选择页 ----------------
   K1 = 下一个(在 [Min,Max] 里循环,跳过 Skip),K2 = 确定。
   Key1_GetStatus()/Key2_GetStatus() 内部已经是"读按下 -> 死等抬手 -> 消抖"
   的完整流程,返回 Press 时手一定松开了,所以这里不用再防连按;
   但两个键必须按 K1 先判、K2 后判,和 Interact() 保持一致。 */
uint8_t Interface_SelectDest(char *Title,char *Label,uint8_t Init,
                             uint8_t Min,uint8_t Max,uint8_t Skip){
	uint8_t v=Init;

	if (v<Min||v>Max) v=Min;
	if (v==Skip){						//初值正好是被禁止的那个,先挪一格
		v++;
		if (v>Max) v=Min;
	}

	while (1){
		OLED_Clear();
		OLED_ShowString(1,1,Title);
		OLED_ShowString(2,1,Label);
		OLED_ShowNum(2,7,v,1);
		OLED_ShowString(3,1,"Range:");
		OLED_ShowNum(3,8,Min,1);
		OLED_ShowString(3,10,"to");
		OLED_ShowNum(3,13,Max,1);
		OLED_ShowString(4,1,"K1:Next K2:OK");

		if (Key1_GetStatus()==Press){	//K1:下一个
			do{
				v++;
				if (v>Max) v=Min;
			}while (v==Skip);
		}else if (Key2_GetStatus()==Press){
			return v;					//K2:确定
		}
	}
}

/* ---------------- 运行页 ----------------
   原来在 Control.c 里的 Nav_ShowRun(),内容原样搬过来。
   Run_Laps 是 Control.h 里的全局量,直接读。 */
void Interface_Run(char *Tag,uint8_t Dest){
	OLED_Clear();
	OLED_ShowString(1,1,"RUN ");
	OLED_ShowString(1,5,Tag);
	OLED_ShowString(2,1,"Dest: ");
	OLED_ShowNum(2,7,Dest,1);
	OLED_ShowString(3,1,"Laps: ");
	OLED_ShowNum(3,7,Run_Laps,3);
	OLED_ShowString(4,1,"LIFT: STOP");
}

/* ---------------- 结果页 ---------------- 原 Nav_ShowResult() */
void Interface_Finish(char *Msg){
	OLED_Clear();
	OLED_ShowString(1,1,"FINISHED");
	OLED_ShowString(2,1,Msg);
	OLED_ShowString(3,1,"Laps: ");
	OLED_ShowNum(3,7,Run_Laps,3);
	OLED_ShowString(4,1,"LIFT: STOP");
}
