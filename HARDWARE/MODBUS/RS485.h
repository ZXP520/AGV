#ifndef __RS485_H
#define __RS485_H
#include "sys.h"



void Timer7_Init(void);
void RS485_Init(void);
void RS485_Service(void);
extern u32 testData1,testData2;
#endif



