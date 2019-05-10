#ifndef __LED_H
#define __LED_H	 
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//LED驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/2
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 
#define L_BK PBout(5)// PB5
#define L_EN PBout(6)// PE6	
#define L_FR PBout(7)// PB7
#define buzzer  PBout(8)// PB8

#define R_BK PEout(3)// PE3	
#define R_EN PEout(4)// PE4
#define R_FR PEout(5)// PE5	
extern u8 Buzzer_flag;
void LED_Init(void);//初始化
void Buzzer(void);
		 				    
#endif
