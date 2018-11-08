#include "timer.h"
#include "led.h"
#include "stm32f10x_tim.h"
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
void TIM4_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //时钟使能
	
	//定时器TIM3初始化
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE ); //使能指定的TIM3中断,允许更新中断

	//中断优先级NVIC设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM7中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器


	TIM_Cmd(TIM4, ENABLE);  //使能TIMx					 
}

u8 Flag_1ms=0,Flag_5ms=0,Flag_10ms=0,Flag_20ms=0,Flag_100ms=0,Flag_500ms=0,Flag_1000ms=0;
//定时器3中断服务程序
void TIM4_IRQHandler(void)   //TIM3中断
{
	static u16 Time_cnt=0;
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)  //检查TIM3更新中断发生与否
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
		
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update );  //清除TIMx更新中断标志 
	}
}


//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void TIM3_PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);   //增加了I2C: RCC_APB1Periph_I2C1
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO, ENABLE);  //使能GPIO外设时钟使能	                                                                     	
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE); //完全重映射 TIM3_CH2->PC6/7/8/9
	
   //设置该引脚为复用输出功能,输出TIM3 CH1\CH2\CH3\CH4的PWM脉冲波形
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 80K
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  不分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = 0; //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 	//TIM_OCPolarity_Low; //输出极性:TIM输出比较极性高
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx CH1 PC6
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx CH2 PC7
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx CH3 PC8
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx CH4 PC9
	
  TIM_CtrlPWMOutputs(TIM3,ENABLE);	//MOE 主输出使能	

	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);  //CH1预装载使能	PC6  
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //CH2预装载使能	PC7
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);  //CH2预装载使能	PC8
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);  //CH2预装载使能	PC9
	
	TIM_ARRPreloadConfig(TIM3, ENABLE); //使能TIMx在ARR上的预装载寄存器
	
	TIM_Cmd(TIM3, ENABLE);  //使能TIM3
	
	TIM_SetCompare1(TIM3,0); //左转
	TIM_SetCompare2(TIM3,0);
	TIM_SetCompare3(TIM3,0);  
	TIM_SetCompare4(TIM3,0);		
}










