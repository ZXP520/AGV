#ifndef __PID_H
#define __PID_H	
#include "sys.h"





void PID_Init(void);
u16 PID_Calculation(u16 set,u16 yout);

#endif


