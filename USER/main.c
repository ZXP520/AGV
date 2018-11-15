#include "sys.h"
#include "led.h"
#include "delay.h"
#include "key.h"
#include "usart.h"
#include "timer.h"
#include "adc.h"
#include "pid.h"
#include "include.h"
#include "runcontrol.h"
#include "RS485.h"

//ALIENTEK战舰STM32开发板实验4
//串口实验  

extern u8 Flag_1ms,Flag_5ms,Flag_10ms,Flag_20ms,Flag_100ms,Flag_500ms,Flag_1000ms; //时间标志
Machine Mach;
static void Stm32_Init(void)
{
	delay_init();	    	 				//延时函数初始化	  
	NVIC_Configuration(); 	 		//设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	TIM3_PWM_Init(1200-1,3-1);	//不分频。PWM频率=72000/3/1200=20Khz
	TIM4_Int_Init(1000-1,72-1); //1M  1ms定时
	RS485_Init();								//MODBUS初始化
	uart_init(115200);				  //串口初始化为115200
	Adc_Init();									//ADC初始化
	PID_Init();									//PID参数初始化
 	LED_Init();			     				//LED端口初始化
	KEY_Init();          				//初始化与按键连接的硬件接口
	TIM_SetCompare1(TIM3,600);
}
 
int main(void)
 {		
	Stm32_Init();
 	while(1)
	{
		
		if(Flag_1ms)  //1MS
		{
			Flag_1ms=0;
			Get_AdcData();	//adc取值1ms一次，5次得出平均值
			RS485_Service();//modbus数据处理，1MS处理一次
		}
		if(Flag_5ms)  //5MS
		{
			Flag_5ms=0;
			
		}
		if(Flag_10ms)	//10MS
		{
			Flag_10ms=0;
			RunControl();
			WorkRunControl();//运行
		}
		if(Flag_20ms)	//20MS
		{
			Flag_20ms=0;
			DealUartCmd();//处理串口命令
		}
		if(Flag_100ms)	//100MS
		{
			Flag_100ms=0;
			
		}
		if(Flag_500ms)	//500MS
		{
			Flag_500ms=0;
			testData1=Mach.adc*3.3/4096*1000;//AD采样数据更新到屏幕
		}
		if(Flag_1000ms)  //1S
		{
			Flag_1000ms=0;
			
		}
	}		
 }

