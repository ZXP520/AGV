#include "timer.h"
#include "led.h"
#include "stm32f10x_tim.h"
#include "control.h"	
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//定时器 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/3
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////   	 

//通用定时器4中断初始化
//这里时钟选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数
//这里使用的是定时器3!

/*
void TIM2_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //时钟使能
	
	//定时器TIM3初始化
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE ); //使能指定的TIM3中断,允许更新中断

	//中断优先级NVIC设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM7中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器


	TIM_Cmd(TIM2, ENABLE);  //使能TIMx					 
}


u8 Flag_1ms=0,Flag_5ms=0,Flag_10ms=0,Flag_20ms=0,Flag_100ms=0,Flag_500ms=0,Flag_1000ms=0;
//定时器3中断服务程序
void TIM2_IRQHandler(void)   //TIM3中断
{
	static u16 Time_cnt=0;
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)  //检查TIM3更新中断发生与否
	{
		Time_cnt++;
		Flag_1ms=1;
		if(Time_cnt%10==0)
		{
			Flag_10ms=1;
		}
		if(Time_cnt%5==0)
		{
			Flag_5ms=1;
		}
		if(Time_cnt%20==0)
		{
			Flag_20ms=1;
		}
		if(Time_cnt%100==0)
		{
			Flag_100ms=1;
		}
		if(Time_cnt%500==0)
		{
			Flag_500ms=1;
		}
		if(Time_cnt%1000==0)
		{
			Flag_1000ms=1;
			Time_cnt=0;
		}
		
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update );  //清除TIMx更新中断标志 
	}
}
*/

//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void TIM8_PWM_Init(u16 arr,u16 psc)
{  
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);   //增加了I2C: RCC_APB1Periph_I2C1
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO, ENABLE);  //使能GPIO外设时钟使能	                                                                     	
	//GPIO_PinRemapConfig(GPIO_Remap_TIM8, ENABLE); //完全重映射 TIM8_CH2->PC6/7/8/9
	
   //设置该引脚为复用输出功能,输出TIM8 CH1\CH2\CH3\CH4的PWM脉冲波形
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 80K
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  不分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = 0; //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; 	//TIM_OCPolarity_Low; //输出极性:TIM输出比较极性高
	TIM_OC1Init(TIM8, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx CH1 PC6
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx CH2 PC7
	TIM_OC3Init(TIM8, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx CH3 PC8
	TIM_OC4Init(TIM8, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx CH4 PC9
	
	
  TIM_CtrlPWMOutputs(TIM8,ENABLE);	//MOE 主输出使能	

	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);  //CH1预装载使能	PB6  
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);  //CH2预装载使能	PB7
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);  //CH2预装载使能	PB8
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);  //CH2预装载使能	PB9
	
	TIM_ARRPreloadConfig(TIM8, ENABLE); //使能TIMx在ARR上的预装载寄存器
	
	
	
	TIM_Cmd(TIM8, ENABLE);  //使能TIM8
	
	
	
	
	TIM_SetCompare1(TIM8,0); //左转
	TIM_SetCompare2(TIM8,0);
	TIM_SetCompare3(TIM8,0);  
	TIM_SetCompare4(TIM8,0);	
		
}




/*
TIM3_Cap_Init(0XFFFF,72-1); //输入捕获1us

u8  TIM3CH1_CAPTURE_STA=0;		    				
u16	TIM3CH1_CAPTURE_VAL;	
u16	TIM3CH1_CAPTURE_VALold;	


u8  TIM3CH2_CAPTURE_STA=0;	   				
u16	TIM3CH2_CAPTURE_VAL;	
u16	TIM3CH2_CAPTURE_VALold;	

u8  TIM3CH3_CAPTURE_STA=0;		    				
u16	TIM3CH3_CAPTURE_VAL;	
u16	TIM3CH3_CAPTURE_VALold;	

u8  TIM3CH4_CAPTURE_STA=0;			    				
u16	TIM3CH4_CAPTURE_VAL;	
u16	TIM3CH4_CAPTURE_VALold;	

*/


/* TIM3输入捕获配置 */
/*
void TIM3_Cap_Init(u16 arr,u16 psc) 
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM3_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //TIM3 时钟使能
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);  //Ê¹ÄÜGPIOAÊ±ÖÓ
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_6;//|GPIO_Pin_7;  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA,GPIO_Pin_6);//|GPIO_Pin_7);						
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0 ;//
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; // 
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB,GPIO_Pin_0);//|GPIO_Pin_7);						
 
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update );     //清除TIM3更新中断标志 
 
	//定时器 TIM3 初始化
	TIM_TimeBaseStructure.TIM_Period = arr; //设置自动重装载寄存器的周期值，使100ms产生一次中断
	TIM_TimeBaseStructure.TIM_Prescaler = psc; //设置预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分频系数
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM 向上计数
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //初始化 TIM3
 
 
  //TIM3输入捕获参数配置
	TIM3_ICInitStructure.TIM_Channel = TIM_Channel_1; //捕获通道IC3
	TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; //上升沿捕获
	TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
	TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; //不分频，每个变化沿都捕获
	TIM3_ICInitStructure.TIM_ICFilter = 0x00;//不滤波
	TIM_ICInit(TIM3, &TIM3_ICInitStructure);
 
  //TIM3输入捕获参数配置
	TIM3_ICInitStructure.TIM_Channel = TIM_Channel_2; //捕获通道IC3
	TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; //上升沿捕获
	TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
	TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; //不分频，每个变化沿都捕获
	TIM3_ICInitStructure.TIM_ICFilter = 0x00;//不滤波
	TIM_ICInit(TIM3, &TIM3_ICInitStructure);
 
	//TIM3输入捕获参数配置
	TIM3_ICInitStructure.TIM_Channel = TIM_Channel_3; //捕获通道IC3
	TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; //上升沿捕获
	TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
	TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; //不分频，每个变化沿都捕获
	TIM3_ICInitStructure.TIM_ICFilter = 0x00;//不滤波
	TIM_ICInit(TIM3, &TIM3_ICInitStructure);
	
		//TIM3输入捕获参数配置
	TIM3_ICInitStructure.TIM_Channel = TIM_Channel_4; //捕获通道IC3
	TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; //上升沿捕获
	TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
	TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; //不分频，每个变化沿都捕获
	TIM3_ICInitStructure.TIM_ICFilter = 0x00;//不滤波
	TIM_ICInit(TIM3, &TIM3_ICInitStructure);
 
	//中断优先级 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn; //TIM3 捕获中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; //从优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ 通道使能
	NVIC_Init(&NVIC_InitStructure); //初始化 NVIC 寄存器
 
	TIM_ITConfig(TIM3,TIM_IT_Update|TIM_IT_CC1|TIM_IT_CC3,ENABLE);       //使能更新中断和捕获中断
 
	TIM_Cmd(TIM3, ENABLE);         //使能定时器
}


u32 LeftEncoder_Cnt=0,RightEncoder_Cnt=0;

//捕获中断处理
void TIM3_IRQHandler(void)
{ 
 //通道1
 	if((TIM3CH1_CAPTURE_STA&0X80)==0)//还未成功捕获
	{	
		if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
		{	    
			if(TIM3CH1_CAPTURE_STA&0X40)//已经捕获到高电平
			{
				if((TIM3CH1_CAPTURE_STA&0X3F)==0X3F)//高电平太长
				{
					TIM3CH1_CAPTURE_STA|=0X80;//标记成功捕获一次
					TIM3CH1_CAPTURE_VAL=0XFFFF;
				}else TIM3CH1_CAPTURE_STA++;
			}	 
		}
		if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)//捕获1发生捕获事件
		{	
			if(TIM3CH1_CAPTURE_STA&0X40)		//捕获到一个上升沿	
			{	  			
				TIM3CH1_CAPTURE_STA|=0X80;		//标记成功捕获下降沿
				
				TIM3CH1_CAPTURE_VAL=TIM_GetCapture1(TIM3);
				if(TIM3CH1_CAPTURE_VAL<TIM3CH1_CAPTURE_VALold)//溢出
				{
					TIM3CH1_CAPTURE_VAL=TIM3CH1_CAPTURE_VAL+0xffff-TIM3CH1_CAPTURE_VALold;
				}
				else //非溢出
				{
					TIM3CH1_CAPTURE_VAL=TIM3CH1_CAPTURE_VAL-TIM3CH1_CAPTURE_VALold;
				}
				//滤除异常干扰
				//if(TIM3CH1_CAPTURE_VAL>200&&TIM3CH1_CAPTURE_VAL<8000)
				{
					LeftEncoder_Cnt++;//计数值增加
				}
				TIM3CH1_CAPTURE_STA=0;//开启下一次捕获
				
				TIM_OC1PolarityConfig(TIM3,TIM_ICPolarity_Rising); //CC1P=0设置为上升沿捕获
			}else  								//还未开始第一次上升沿捕获
			{
				TIM3CH1_CAPTURE_STA=0;			//清零
				TIM3CH1_CAPTURE_VAL=0;
				TIM3CH1_CAPTURE_VALold=	TIM_GetCapture1(TIM3);//取捕获寄存器的值
				TIM3CH1_CAPTURE_STA|=0X40;		//标记捕获到了上升沿
				TIM_OC1PolarityConfig(TIM3,TIM_ICPolarity_Falling); //CC1P=0设置为下降捕获
			}		    
		}			     	    					   
 	}
 
	
 //通道2
	if((TIM3CH2_CAPTURE_STA&0X80)==0)     
	{         
		if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)                
		{            
			if(TIM3CH2_CAPTURE_STA&0X40)
			{
				if((TIM3CH2_CAPTURE_STA&0X3F)==0X3F)
				{
								TIM3CH2_CAPTURE_STA|=0X80;
								TIM3CH2_CAPTURE_VAL=0XFFFF;
				}else TIM3CH2_CAPTURE_STA++;
			}         
		}
		if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)
		{        
			if(TIM3CH2_CAPTURE_STA&0X40)                              
			{                                 
				TIM3CH2_CAPTURE_STA|=0X80;               
				TIM3CH2_CAPTURE_VAL=TIM_GetCapture2(TIM3);
				
				if(TIM3CH2_CAPTURE_VAL<TIM3CH2_CAPTURE_VALold)//溢出
				{
					TIM3CH2_CAPTURE_VAL=TIM3CH2_CAPTURE_VAL+0xffff-TIM3CH2_CAPTURE_VALold;
				}
				else //非溢出
				{
					TIM3CH2_CAPTURE_VAL=TIM3CH2_CAPTURE_VAL-TIM3CH2_CAPTURE_VALold;
				}
				
				TIM_OC2PolarityConfig(TIM3,TIM_ICPolarity_Rising); 
			}else                                                                  
			{
				TIM3CH2_CAPTURE_STA=0;                        
				TIM3CH2_CAPTURE_VAL=0;
				//TIM_SetCounter(TIM3,0); 
				TIM3CH2_CAPTURE_VALold=	TIM_GetCapture2(TIM3);//取捕获寄存器的值
				TIM3CH2_CAPTURE_STA|=0X40;                     
				TIM_OC2PolarityConfig(TIM3,TIM_ICPolarity_Falling); //CC1P=0设置为下降捕获
			}                    
		}                                                                                    
	}

	//通道3
	if((TIM3CH3_CAPTURE_STA&0X80)==0)
	{	 
		if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)		 
		{	    
			if(TIM3CH3_CAPTURE_STA&0X40)
			{
				if((TIM3CH3_CAPTURE_STA&0X3F)==0X3F)
				{
					TIM3CH3_CAPTURE_STA|=0X80;
					TIM3CH3_CAPTURE_VAL=0XFFFF;
				}else TIM3CH3_CAPTURE_STA++;
			}	 
		}
		if (TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET)
		{	
			if(TIM3CH3_CAPTURE_STA&0X40)		
			{	  			
				TIM3CH3_CAPTURE_STA|=0X80;		
				
				
				
				TIM3CH3_CAPTURE_VAL=TIM_GetCapture3(TIM3);
				
				if(TIM3CH3_CAPTURE_VAL<TIM3CH3_CAPTURE_VALold)
				{
					TIM3CH3_CAPTURE_VAL=TIM3CH3_CAPTURE_VAL+0xffff-TIM3CH3_CAPTURE_VALold;
				}
				else
				{
					TIM3CH3_CAPTURE_VAL=TIM3CH3_CAPTURE_VAL-TIM3CH3_CAPTURE_VALold;
				}
				
				//滤除异常干扰
				//if(TIM3CH3_CAPTURE_VAL>200&&TIM3CH3_CAPTURE_VAL<8000)
				{
					RightEncoder_Cnt++;//计数值增加
					
				}
				TIM3CH3_CAPTURE_STA=0;//开启下一次捕获
				
				TIM_OC3PolarityConfig(TIM3,TIM_ICPolarity_Rising); 
			}else  								
			{
				TIM3CH3_CAPTURE_STA=0;			
				TIM3CH3_CAPTURE_VAL=0;
				TIM3CH3_CAPTURE_VALold=	TIM_GetCapture3(TIM3);
				TIM3CH3_CAPTURE_STA|=0X40;
				TIM_OC3PolarityConfig(TIM3,TIM_ICPolarity_Falling); //CC1P=0设置为下降捕获
			}		    
		}			     	    					   
 	}
 
 
	//通道4
	if((TIM3CH4_CAPTURE_STA&0X80)==0)      
	{         
		if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)                
		{            
			if(TIM3CH4_CAPTURE_STA&0X40)
			{
				if((TIM3CH4_CAPTURE_STA&0X3F)==0X3F)
				{
								TIM3CH4_CAPTURE_STA|=0X80;
								TIM3CH4_CAPTURE_VAL=0XFFFF;
				}else TIM3CH4_CAPTURE_STA++;
			}         
		}
		if (TIM_GetITStatus(TIM3, TIM_IT_CC4) != RESET)
		{        
			if(TIM3CH4_CAPTURE_STA&0X40)                       
			{                                 
				TIM3CH4_CAPTURE_STA|=0X80;             
				TIM3CH4_CAPTURE_VAL=TIM_GetCapture4(TIM3);
				
				if(TIM3CH4_CAPTURE_VAL<TIM3CH4_CAPTURE_VALold)
				{
					TIM3CH4_CAPTURE_VAL=TIM3CH4_CAPTURE_VAL+0xffff-TIM3CH4_CAPTURE_VALold;
				}
				else
				{
					TIM3CH4_CAPTURE_VAL=TIM3CH4_CAPTURE_VAL-TIM3CH4_CAPTURE_VALold;
				}
				
				TIM_OC4PolarityConfig(TIM3,TIM_ICPolarity_Rising);
			}else                                                              
			{
				TIM3CH4_CAPTURE_STA=0;                      
				TIM3CH4_CAPTURE_VAL=0;
				//TIM_SetCounter(TIM3,0);
				TIM3CH4_CAPTURE_VALold=TIM_GetCapture4(TIM3);
				TIM3CH4_CAPTURE_STA|=0X40;  
				TIM_OC4PolarityConfig(TIM3,TIM_ICPolarity_Falling); //CC1P=0设置为下降捕获
			}                    
		}                                                                                    
	}
	
   TIM_ClearITPendingBit(TIM3, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4|TIM_IT_Update); //清除中断标志
	 
}
 
*/

void TIM3_Configuration(void)//编码器接口设置TIM3/PA6-A相  PA7-B相
{
  GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);  
	                                                       				
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;          
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	   
	GPIO_Init(GPIOA, &GPIO_InitStructure);				 
	GPIO_WriteBit(GPIOA, GPIO_Pin_6,Bit_SET);
	GPIO_WriteBit(GPIOA, GPIO_Pin_7,Bit_SET); 

  TIM_TimeBaseStructure.TIM_Period = EncoderPeriod; 
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;    
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); 
	
	//设置定时器3位编码器模式 IT1 IT2位上升沿计数
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12,TIM_ICPolarity_BothEdge,TIM_ICPolarity_BothEdge);
	TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = 8;      
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
  TIM_ClearFlag(TIM3, TIM_FLAG_Update);     
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); 
  TIM3->CNT = 0;
	TIM_Cmd(TIM3, ENABLE);
}

void TIM5_Configuration(void)//编码器接口设置TIM5/PA0-A相  PA1-B相
{
  GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE); 
	                                                       				
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 
	GPIO_Init(GPIOA, &GPIO_InitStructure);				 
	GPIO_WriteBit(GPIOA, GPIO_Pin_0,Bit_SET);
	GPIO_WriteBit(GPIOA, GPIO_Pin_1,Bit_SET); 

  TIM_TimeBaseStructure.TIM_Period = EncoderPeriod;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); 
	
	//设置定时器5位编码器模式 IT1 IT2位上升沿计数
	TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12,TIM_ICPolarity_BothEdge,TIM_ICPolarity_BothEdge);
	TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = 8;  
  TIM_ICInit(TIM5, &TIM_ICInitStructure);
  TIM_ClearFlag(TIM5, TIM_FLAG_Update); 
  TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
  TIM5->CNT = 0;
	TIM_Cmd(TIM5, ENABLE);
}

//中断处理函数在stm32f10x_it.c里面
static void SysTick_init(u8 u_ms )
{  
	 u16 fac_us,fac_ms;
   SysTick->CTRL&=0xfffffffb;    //bit2清空,选择外部时钟  HCLK/8
   fac_us=SystemCoreClock/8000000;		//每秒钟的计数次数 单位为K	      
   fac_ms=(u16)fac_us*1000;

   SysTick->LOAD=(u32)u_ms*fac_ms;    //时间加载(SysTick->LOAD为24bit)
   SysTick->VAL =0x00;                    //清空计数器
   SysTick->CTRL |=(1<<0 |1<<1);    //开始倒数    
    /* Function successful */
}


void Time_Config(void)
{
	SysTick_init(1);//开启1ms的系统滴答定时器
	TIM3_Configuration();
	TIM5_Configuration();
	TIM8_PWM_Init(TIM8_Period-1,12-1);	//不分频。PWM频率=72000/5/1200=12Khz
}



