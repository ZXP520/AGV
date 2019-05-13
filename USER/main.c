#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "timer.h"
#include "include.h"
#include "led.h"
#include "runcontrol.h"
#include "control.h"

#include "mpu6050.h"
#include "kalman.h"
#include "dealdata.h"
//ALIENTEK战舰STM32开发板实验4
//串口实验  

extern u8 Flag_1ms,Flag_5ms,Flag_10ms,Flag_20ms,Flag_100ms,Flag_500ms,Flag_1000ms; //时间标志
Machine Mach;
static void Stm32_Init(void)
{
	delay_init();	    	 				//延时函数初始化	  
	NVIC_Configuration(); 	 		//设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	TIM8_PWM_Init(TIM8_Period-1,5-1);	//不分频。PWM频率=72000/5/1200=12Khz
	TIM2_Int_Init(1000-1,72-1); //1M  1ms定时
	
	TIM3_Cap_Init(0XFFFF,72-1); //输入捕获1us
	
	uart_init(115200);				  //串口初始化为115200
	USART2_init(115200);
	
	I2C_MPU6050_Init();         //IIC³õÊ¼»¯
	InitMPU6050();              //MPU6050³õÊ¼»¯

	LeftWheelSpeedSet(200);
	RightWheelSpeedSet(200);
	
	
	//LED_Init();
	
	
}
 
int main(void)
 {		
	static u8 start_flag=0;
	Stm32_Init();
 	while(1)
	{
		
		if(Flag_1ms)  //1MS
		{
			
			Flag_1ms=0;
			
		}
		if(Flag_5ms)  //5MS
		{
		  //ReadWheelCnt();
			
			Flag_5ms=0;
			
		}
		if(Flag_10ms)	//10MS
		{
			Flag_10ms=0;
			RunWheelcontrol();//速度控制
			
		}
		if(Flag_20ms)	//20MS
		{
			Angle_Calcu();
			//SendData_To_Ros();
			Flag_20ms=0;
		}
		if(Flag_100ms)	//100MS
		{
			
			
			Flag_100ms=0;		
		}
		if(Flag_500ms)	//500MS
		{
			
			Flag_500ms=0;
		}
		if(Flag_1000ms)  //1S
		{
			//TestSendData_To_Ros();
			
			Flag_1000ms=0;

		}
	}		
 }

 
 
 
 
