#include "control.h"		
#include "sys.h"
#include "include.h"
#include "led.h"
#include "timer.h"
#include "usart.h"
#include "Encoder.h"
/**************************************************************************
函数功能：轮子速度设置 mm/s
入口参数：direction(1为前进，0为后退)
					speed     （速度mm/s）34mm/s-376mm/s  精度为34mm/s
	
**************************************************************************/
Wheel LeftWheel,RightWheel,AllWheel;//定义左右轮结构体


//左轮速度设置
void LeftWheelSpeedSet(int speed)
{
	if(speed>=0)//正方向
	{
		if(speed>MAXSPEED){speed=MAXSPEED;}//限速
		LeftWheel.Direct=1;
	}
	else        //反方向
	{
		speed=-speed;
		if(speed>MAXSPEED){speed=MAXSPEED;}//限速
		LeftWheel.Direct=0;
	}
	
	
	LeftWheel.AimsEncoder=speed*SPEED_TO_ENCODER+0.5;//+0.5四舍五入
}

//右轮速度设置
void  RightWheelSpeedSet(int speed)
{
	if(speed>=0)//正方向
	{
		if(speed>MAXSPEED){speed=MAXSPEED;}//限速
		RightWheel.Direct=1;
	}
	else        //反方向
	{
		speed=-speed;
		if(speed>MAXSPEED){speed=MAXSPEED;}//限速
		RightWheel.Direct=0;
	}
	RightWheel.AimsEncoder=speed*SPEED_TO_ENCODER+0.5;//+0.5四舍五入
}


/**************************************************************************
函数功能：PID运动控制
					10ms进一次
					在系统的滴答定时器中调用
**************************************************************************/
PID_AbsoluteType PID_Control;//定义PID算法的结构体


void RunWheelcontrol(void)
{	
	static u8 cnt=0;
	float temp=0;
	static float speed_usart=0;
	cnt++;
	
	//停车标志 进入抱死状态
	if(AllWheel.stop_flag)
	{
		TIM_SetCompare1(TIM8,TIM8_Period); 
		TIM_SetCompare2(TIM8,TIM8_Period);
		TIM_SetCompare3(TIM8,TIM8_Period); 
		TIM_SetCompare4(TIM8,TIM8_Period); 
		return;
	}
	
	//获得PID调速后的PWM
	LeftWheel.MotoPwm =myabs( LeftIncremental_PI(abs(GetEncoder.V5) ,LeftWheel.AimsEncoder ));//获得PID调速后的PWM
	RightWheel.MotoPwm=myabs(RightIncremental_PI(abs(GetEncoder.V3) ,RightWheel.AimsEncoder));
	
	Xianfu_Pwm();//限幅
	
	if(cnt%10==0)
	{
		//u2_printf("PWM:	%d   Right:	%d	PWM:	%d  Left:	%d	aim:%d\r\n",RightWheel.MotoPwm,abs(GetEncoder.V3),LeftWheel.MotoPwm,abs(GetEncoder.V5),LeftWheel.AimsEncoder);
		temp=GetEncoder.V3;
		printf("@%d@",(int)(temp/0.11));
	}
	
	//设置PWM与方向
	SetLeft_Pwm (LeftWheel.MotoPwm  ,LeftWheel.Direct );
	SetRight_Pwm(RightWheel.MotoPwm ,RightWheel.Direct);
	
}


/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：PWM mode(1为前进， 0为后退)
返回  值：无
**************************************************************************/
void SetLeft_Pwm(int moto,u8 mode)
{
	if(mode)
	{
		TIM_SetCompare1(TIM8,moto); 
		TIM_SetCompare2(TIM8,0); 
	}
	else
	{
		TIM_SetCompare1(TIM8,0); 
		TIM_SetCompare2(TIM8,moto); 
	}
}

void SetRight_Pwm(int moto,u8 mode)
{
	if(mode)
	{
		TIM_SetCompare3(TIM8,0); 
		TIM_SetCompare4(TIM8,moto); 
	}
	else
	{	
		TIM_SetCompare3(TIM8,moto); 
		TIM_SetCompare4(TIM8,0); 
	}
}



/**************************************************************************
函数功能：限制PWM赋值 
入口参数：无
返回  值：无
**************************************************************************/

void Xianfu_Pwm(void)
{	
	  //TIM8_Period=1200;    //===PWM满幅是1200 限制在1200
	  if(LeftWheel.MotoPwm<-TIM8_Period)  LeftWheel.MotoPwm=-TIM8_Period;	
		if(LeftWheel.MotoPwm>TIM8_Period)   LeftWheel.MotoPwm=TIM8_Period;	

		if(RightWheel.MotoPwm<-TIM8_Period) RightWheel.MotoPwm=-TIM8_Period;	
		if(RightWheel.MotoPwm>TIM8_Period)  RightWheel.MotoPwm=TIM8_Period;	
}

/**************************************************************************
函数功能：绝对值函数
入口参数：int
返回  值：unsigned int
**************************************************************************/
int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=0;//temp=-a;  
	  else temp=a;
	  return temp;
}




float Amplitude_PKP=20,Amplitude_PKI=0.1,Amplitude_PKD=25,Amplitude_VKP=2,Amplitude_VKI=3; //PID调试相关参数


/**************************************************************************
函数功能：增量PI控制器
入口参数：编码器测量值，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
//左PID
float LVelocity_KP=120,LVelocity_KI=5;

int LeftIncremental_PI (int Encoder,int Target)
{ 	
	 static float Bias=0,Pwm=0,Last_bias=0;
	 Bias=Target-Encoder;                                  //计算偏差
	 Pwm+=LVelocity_KP*(Bias-Last_bias)+LVelocity_KI*Bias;   //增量式PI控制器
	 if(Pwm>1200){Pwm=1200;}
	 else if(Pwm<0){Pwm=0;}
	 Last_bias=Bias;	                                     //保存上一次偏差 
	 return Pwm;                                           //增量输出

}

//右PID
float RVelocity_KP=120,RVelocity_KI=5;

int RightIncremental_PI (int Encoder,int Target)
{ 	
	 static float Bias=0,Pwm=0,Last_bias=0;
	 Bias=Target-Encoder;                                  //计算偏差
	 Pwm+=RVelocity_KP*(Bias-Last_bias)+RVelocity_KI*Bias;   //增量式PI控制器
	 if(Pwm>1200){Pwm=1200;}
	 else if(Pwm<0){Pwm=0;}
	 Last_bias=Bias;	                                     //保存上一次偏差 
	 return Pwm;                                           //增量输出
}
/**************************************************************************
函数功能：位置式PID控制器
入口参数：编码器测量位置信息，目标位置
返回  值：电机PWM
根据位置式离散PID公式 
pwm=Kp*e(k)+Ki*∑e(k)+Kd[e（k）-e(k-1)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  
∑e(k)代表e(k)以及之前的偏差的累积和;其中k为1,2,,k;
pwm代表输出
**************************************************************************/
float Position_KP=20,Position_KI=2,Position_KD=0;      //PID系数
int Position_PID (int Encoder,int Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Target-Encoder;                                  //计算偏差
	 Integral_bias+=Bias;	                                 //求出偏差的积分
	 Pwm=Position_KP*Bias+Position_KI*Integral_bias+Position_KD*(Bias-Last_Bias);       //位置式PID控制器
	 Last_Bias=Bias;                                       //保存上一次偏差 
	 return Pwm;                                           //增量输出
}


//绝对式PID算法
void PID_AbsoluteMode(PID_AbsoluteType* PID)
{
 if(PID->kp      < 0)    PID->kp      = -PID->kp;
 if(PID->ki      < 0)    PID->ki      = -PID->ki;
 if(PID->kd      < 0)    PID->kd      = -PID->kd;
 if(PID->errILim < 0)    PID->errILim = -PID->errILim;

 PID->errP = PID->errNow;  //读取现在的误差，用于kp控制

 PID->errI += PID->errNow; //误差积分，用于ki控制

 if(PID->errILim != 0)	   //微分上限和下限
 {
  if(     PID->errI >  PID->errILim)    PID->errI =  PID->errILim;
  else if(PID->errI < -PID->errILim)    PID->errI = -PID->errILim;
 }
 
 PID->errD = PID->errNow - PID->errOld;//误差微分，用于kd控制

 PID->errOld = PID->errNow;	//保存现在的误差
 
 PID->ctrOut = PID->kp * PID->errP + PID->ki * PID->errI + PID->kd * PID->errD;//计算绝对式PID输出

}
