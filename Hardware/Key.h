#ifndef Key_H
#define Key_H

enum Key_Status{
	Release=0,
	Press
};

void Key_Init(void);
enum Key_Status Key1_GetStatus(void);
enum Key_Status Key2_GetStatus(void);

#endif
