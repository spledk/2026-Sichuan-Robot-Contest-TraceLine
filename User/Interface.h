#ifndef Interface_H
#define Interface_H

#include <stdint.h>

/* 第一层选择界面的页号。Control.c 里的 Interact() 只管
   "K1 顺序翻页 / K2 按页号调对应流程",一幅画面都不画。 */
enum Interface_Page{
	Page0_MainInterface=0,
	Page1_BackForth,			//基本(1) 0号<->3号 折返,不用选目的地
	Page2_GoDestination,		//基本(2) 0号->目的地->0号,要选 1 个
	Page3_A2B,					//发挥(1) 0号->A->B->0号,要连选 2 个
	Page4_OffLine,				//发挥(2) 巡线到1号后离线开到停车点,要选 1 个
	Page5_Back2MainInterface	//翻到这一页回主页
};

void Interface(enum Interface_Page Page);

/* 目的地选择页:进去先画一次,之后 K1 = 下一个 K2 = 确定,返回确认的编号。
     Title  : 第 1 行标题(<=16 字符)
     Label  : 第 2 行的名字,如 "Dest"/"A"/"B"/"Stop"(值固定画在第 7 列)
     Init   : 进去时显示的初值,不在 [Min,Max] 内会自动夹到 Min
     Min/Max: 这次允许循环的范围,含端点
     Skip   : 不允许被选中的编号(A->B 的 B 不能等于 A),没有就传 0xFF
   调用者拿到的一定是 [Min,Max] 内且不等于 Skip 的值,模式函数里的夹取
   只是兜底,正常情况下不会触发。 */
uint8_t Interface_SelectDest(char *Title,char *Label,uint8_t Init,
                             uint8_t Min,uint8_t Max,uint8_t Skip);

/* 运行中显示当前目的地 —— 题目 三.2.(4) 要求"行驶过程需用 OLED 显示当前目的地"。
   Tag 是阶段标签,不超过 4 个字符("GO"/"BACK"/"REV"/"GO A"/...)。 */
void Interface_Run(char *Tag,uint8_t Dest);

/* 一个模式跑完后的结果页,Msg 是结果字符串(如 "OK"/"ERR LOST LINE")。 */
void Interface_Finish(char *Msg);

#endif
