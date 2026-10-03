#ifndef Interface_H
#define Interface_H

enum Interface_Page{
	Page0_MainInterface=0,
	Page1_BackForth,
	Page2_GoDestination,
	Page3_A2B,
	Page4_OffLine,
	Page5_ShowDestination,
	Page6_Config,
	Page7_Back2MainInterface
};

void Interface(enum Interface_Page Page);

#endif
