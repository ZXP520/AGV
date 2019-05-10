#include "sys.h"
#include "include.h"
#include "runcontrol.h"
#include "led.h"
#include "timer.h"
#include "usart.h"
#include "control.h"
u32 EconedCnt=0;

void Wheelcontrol(void)
{	
	
	

	
	
	
	
}

void ReadWheelCnt(void)
{
		u32 temp=0; 
		if(TIM3CH1_CAPTURE_STA&0X80)//成功捕获一次周期
		{
			temp=TIM3CH1_CAPTURE_STA&0X3F;
			//temp*=65536;//溢出时间总和  //计数器没清零所以不能算溢出值
			temp+=TIM3CH1_CAPTURE_VAL;//得到周期时长
			u2_printf("HIGH1:%d us\r\n",temp);//打印周期时间
			TIM3CH1_CAPTURE_STA=0;//开启下一次捕获
		}
		if(TIM3CH2_CAPTURE_STA&0X80)
        {
			temp=TIM3CH2_CAPTURE_STA&0X3F;
			//temp*=65536;
			temp+=TIM3CH2_CAPTURE_VAL;
			u2_printf("HIGH2:%d us\r\n",temp);
			TIM3CH2_CAPTURE_STA=0;
  	    }
 		if(TIM3CH3_CAPTURE_STA&0X80)
		{
			temp=TIM3CH3_CAPTURE_STA&0X3F;
			//temp*=65536;
			temp+=TIM3CH3_CAPTURE_VAL;
			EconedCnt=temp;
			u2_printf("HIGH3:%d us\r\n",temp);
			TIM3CH3_CAPTURE_STA=0;
		}
		if(TIM3CH4_CAPTURE_STA&0X80)//
		{
			temp=TIM3CH4_CAPTURE_STA&0X3F;
			//temp*=65536;
			temp+=TIM3CH4_CAPTURE_VAL;
			u2_printf("HIGH4:%d us\r\n",temp);
			TIM3CH4_CAPTURE_STA=0;
		}
}








