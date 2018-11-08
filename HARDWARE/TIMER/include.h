#ifndef __INCLUDE_H
#define __INCLUDE_H
#include "sys.h"
#include "stm32f10x_tim.h"
#include "adc.h"
#include "stdio.h"

typedef struct 
{
	u16 adc;
	u8 	key1;
	u8  key2;
	u8  ResetOK;
	enum 
	{
		Run=0,				//运行
	  Reset 				//复位
	
	}WorkStatus;//机器工作状态运行、复位
	enum 
	{
		Endend=0,		  //结束
		Ending,    		//准备结束
		Starting,			//准备开始
		Startend,  		//开始
	 	Stop			 		//暂停
	  	
	}Status;//机器状态开始、暂停、结束
}Machine;
	
extern Machine Mach;

#endif



