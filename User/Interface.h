#ifndef Interface_H
#define Interface_H

#include <stdint.h>

/* 交互界面(人机界面)的页面表。
   页面的绘制和按键处理都已经搬到 Control.c 里(Interact / Page_Show),
   这里只留页面枚举,给 Control.c 用。 */

enum Interface_Page{
	Page_Run=0,			//开始比赛
	Page_Gyro,			//配置1:陀螺仪零偏校准
	Page_Meter,			//配置2:1m 里程标定
	Page_Count			//页数(不是页面),K1 翻到这里就回 Page_Run
};

#endif
