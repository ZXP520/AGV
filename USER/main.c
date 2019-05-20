#include "sys.h"
#include "delay.h"
#include "timer.h"
#include "include.h"
#include "led.h"
#include "runcontrol.h"
#include "control.h"
#include "gy85.h"
#include "kalman.h"
#include "dealdata.h"
#include "stm32f10x_it.h" 
#include "Encoder.h"
#include "bsp_usart.h"
#include "usart.h"


//时间标志
extern u32 Time_cnt;


static void Stm32_Init(void)
{
	NVIC_Configuration(); 	 		//设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uart_init(115200);				  //串口初始化为115200
	USART2_Config(115200);			//串口初始化为115200
	Time_Config();							//定时器初始化
	InitGY85();                 //陀螺仪初始化

	//LeftWheelSpeedSet(0);
	//RightWheelSpeedSet(0);
	//ThreeWheelSpeedSet(0);
	//SetLeft_Pwm(300,0);
	//SetRight_Pwm(300,0);
	//SetThree_Pwm(300,0);
	//LeftWheel.MotoPwm=1200;
	//RightWheel.MotoPwm=1200;
	//LeftWheelSpeedSet(-200);
	//RightWheelSpeedSet(-200);
	
	
}
 
int main(void)
 {		
	static u32 Time_1MS=0,Time_5MS=0,Time_10MS=0,Time_20MS=0,Time_100MS=0,Time_500MS=0,Time_1000MS=0;
	static u8 time_cnt=0;
	Stm32_Init();
	AllWheel.imu_num=9;
 	while(1)
	{
		
		if(Time_cnt-Time_1MS>=1)
		{
			
			Time_1MS=Time_cnt;
		}
		
		if(Time_cnt-Time_5MS>=5)
		{
			
			Time_5MS=Time_cnt;
		}
		
		if(Time_cnt-Time_10MS>=10)
		{
			//PID调速已经放到滴答定时器中断中不会被别的干扰
			Time_10MS=Time_cnt;
		}
		
		if(Time_cnt-Time_20MS>=20)
		{
			//20ms发一次数据给ros
			SendData_To_Ros();
			//TestSendData_To_Ros();
			Time_20MS=Time_cnt;
		}
		
		if(Time_cnt-Time_100MS>=100)
		{
		  printf("PWM:%d  Right:%d	PWM:%d  Left:%d	PWM:%d	Three:%d	aim:%d\r\n",RightWheel.MotoPwm,abs(GetEncoder.V3),LeftWheel.MotoPwm,abs(GetEncoder.V5),
			ThreeWheel.MotoPwm,abs(GetEncoder.V4),LeftWheel.AimsEncoder);
			Time_100MS=Time_cnt;
		}
		if(Time_cnt-Time_500MS>500)
		{
			
			Time_500MS=Time_cnt;
		}
		if(Time_cnt-Time_1000MS>=1000)
		{
			
			if(time_cnt<5)
			{
				time_cnt++;
			}
			if(time_cnt==5)
			{
				
				OmniWheelscontrol(100,0,0,0);
				//LeftWheelSpeedSet(200);
				//RightWheelSpeedSet(200);
				//ThreeWheelSpeedSet(200);
				time_cnt=6;
			}
			Time_1000MS=Time_cnt;
		}
	}	
 }

 
 
 
 
