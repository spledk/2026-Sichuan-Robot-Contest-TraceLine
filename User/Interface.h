#ifndef Interface_H
#define Interface_H

#include <stdint.h>


enum Interface_Page{
	Page0_MainInterface=0,
	Pageend,
};

void Interface(enum Interface_Page Page);


#endif
