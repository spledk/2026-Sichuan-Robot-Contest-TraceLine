#include "stm32f10x.h"
#include "Interface.h"
#include "Control.h"
#include "OLED.h"

/* OLED 是 128x64 8x16 字库:Line 1~4、Column 1~16,而且库里没有中文字模,
   所以界面全部用 ASCII,每行不超过 16 个字符。 */

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
			OLED_ShowString(2,1,"0 <-> 3  (auto)");
			OLED_ShowString(3,1,"Laps: ");
			OLED_ShowNum(3,7,Run_Laps,3);
			OLED_ShowString(4,1,"K1:Page K2:Start");
			break;

		case Page2_GoDestination:
			OLED_Clear();
			OLED_ShowString(1,1,"2.GO & RETURN");
			OLED_ShowString(2,1,"Dest: ");
			OLED_ShowNum(2,7,Dest_Main,1);
			OLED_ShowString(3,1,"0 <-> n <-> 0");
			OLED_ShowString(4,1,"K1:Page K2:Start");
			break;

		case Page3_A2B:
			OLED_Clear();
			OLED_ShowString(1,1,"3.A TO B");
			OLED_ShowString(2,1,"A:");
			OLED_ShowNum(2,3,Dest_Main,1);
			OLED_ShowString(2,5,"B:");
			OLED_ShowNum(2,7,Dest_Alt,1);
			OLED_ShowString(3,1,"stop 2s each");
			OLED_ShowString(4,1,"K1:Page K2:Start");
			break;

		case Page4_OffLine:
			OLED_Clear();
			OLED_ShowString(1,1,"4.OFF LINE");
			OLED_ShowString(2,1,"Trace to 1");
			OLED_ShowString(3,1,"Stop at: ");
			OLED_ShowNum(3,10,Dest_Alt,1);
			OLED_ShowString(4,1,"K1:Page K2:Start");
			break;

		case Page5_ShowDestination:
			OLED_Clear();
			OLED_ShowString(1,1,"5.DESTINATION");
			OLED_ShowString(2,1,"Dest: ");
			OLED_ShowNum(2,7,Dest_Main,1);
			OLED_ShowString(3,1,"K2:Next");
			OLED_ShowString(4,1,"K1:Page");
			break;

		case Page6_Config:
			OLED_Clear();
			OLED_ShowString(1,1,"6.CONFIG");
			OLED_ShowString(2,1,"Alt : ");
			OLED_ShowNum(2,7,Dest_Alt,1);
			OLED_ShowString(3,1,"K2:Next");
			OLED_ShowString(4,1,"K1:Page");
			break;

		case Page7_Back2MainInterface:
			OLED_Clear();
			OLED_ShowString(1,1,"7.BACK");
			OLED_ShowString(2,1,"K1:Main");
			break;
	}
}
