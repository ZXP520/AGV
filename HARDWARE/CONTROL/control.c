#include "control.h"		
#include "sys.h"
#include "include.h"
#include "led.h"
#include "timer.h"
#include "usart.h"
#include <math.h>
#include "Encoder.h"


Wheel LeftWheel,RightWheel,ThreeWheel,AllWheel;//���������ֽṹ��

/*******************************************************************************
* Function Name  : LeftWheelSpeedSet
* Description    : �����ٶ�����
* Input          : �����ٶ� 
* Output         : None
* Return         : None 
****************************************************************************** */
void LeftWheelSpeedSet(int speed)
{
	if(speed>=0)//������
	{
		if(speed>MAXSPEED){speed=MAXSPEED;}//����
		LeftWheel.Direct=1;
	}
	else        //������
	{
		speed=-speed;
		if(speed>MAXSPEED){speed=MAXSPEED;}//����
		LeftWheel.Direct=0;
	}
	
	
	LeftWheel.AimsEncoder=speed*SPEED_TO_ENCODER+0.5;//+0.5��������
}

/*******************************************************************************
* Function Name  : LeftWheelSpeedSet
* Description    : �����ٶ�����
* Input          : �����ٶ� 
* Output         : None
* Return         : None 
****************************************************************************** */
void  RightWheelSpeedSet(int speed)
{
	if(speed>=0)//������
	{
		if(speed>MAXSPEED){speed=MAXSPEED;}//����
		RightWheel.Direct=1;
	}
	else        //������
	{
		speed=-speed;
		if(speed>MAXSPEED){speed=MAXSPEED;}//����
		RightWheel.Direct=0;
	}
	RightWheel.AimsEncoder=speed*SPEED_TO_ENCODER+0.5;//+0.5��������
}

/*******************************************************************************
* Function Name  : LeftWheelSpeedSet
* Description    : �����ٶ�����
* Input          : �����ٶ� 
* Output         : None
* Return         : None 
****************************************************************************** */
void  ThreeWheelSpeedSet(int speed)
{
	if(speed>=0)//������
	{
		if(speed>MAXSPEED){speed=MAXSPEED;}//����
		ThreeWheel.Direct=1;
	}
	else        //������
	{
		speed=-speed;
		if(speed>MAXSPEED){speed=MAXSPEED;}//����
		ThreeWheel.Direct=0;
	}
	ThreeWheel.AimsEncoder=speed*SPEED_TO_ENCODER+0.5;//+0.5��������
}


/**************************************************************************
�������ܣ�PID�˶�����
					10ms��һ��
					������2�е���
**************************************************************************/
void RunWheelcontrol(void)
{	
	float temp=0;
	
	//ͣ����־ ���뱧��״̬
	if(AllWheel.stop_flag)
	{
		TIM_SetCompare1(TIM8,0); 
		TIM_SetCompare2(TIM8,0);
		TIM_SetCompare3(TIM8,0); 
		TIM_SetCompare4(TIM8,0); 
		
		TIM_SetCompare1(TIM1,0); 
		TIM_SetCompare2(TIM1,0); 
		TIM_SetCompare3(TIM1,0); 
		TIM_SetCompare4(TIM1,0); 
		return;
	}
	
	//���PID���ٺ��PWM
#if VERSION==1
	LeftWheel.MotoPwm =myabs( LeftIncremental_PI(abs(GetEncoder.V3) ,LeftWheel.AimsEncoder ));//���PID���ٺ��PWM
	RightWheel.MotoPwm=myabs(RightIncremental_PI(abs(GetEncoder.V4) ,RightWheel.AimsEncoder));
	ThreeWheel.MotoPwm=myabs(ThreeIncremental_PI(abs(GetEncoder.V5) ,ThreeWheel.AimsEncoder));
#else
	LeftWheel.MotoPwm =myabs( LeftIncremental_PI(abs(GetEncoder.V5) ,LeftWheel.AimsEncoder ));//���PID���ٺ��PWM
	RightWheel.MotoPwm=myabs(RightIncremental_PI(abs(GetEncoder.V3) ,RightWheel.AimsEncoder));
#endif
	
	Xianfu_Pwm();//�޷�

	//����PWM�뷽��
	SetLeft_Pwm (LeftWheel.MotoPwm  ,LeftWheel.Direct );
	SetRight_Pwm(RightWheel.MotoPwm ,RightWheel.Direct);
	SetThree_Pwm(ThreeWheel.MotoPwm ,ThreeWheel.Direct);
	
}


/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ���
��ڲ�����PWM mode(1Ϊǰ���� 0Ϊ����)
����  ֵ����
**************************************************************************/
void SetLeft_Pwm(int moto,u8 mode)
{
	if(mode)
	{
		TIM_SetCompare1(TIM8,TIM8_Period-moto); 
		TIM_SetCompare2(TIM8,TIM8_Period); 
		
	}
	else
	{
		TIM_SetCompare1(TIM8,TIM8_Period); 
		TIM_SetCompare2(TIM8,TIM8_Period-moto); 
		
	}
}
/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ���
��ڲ�����PWM mode(1Ϊǰ���� 0Ϊ����)
����  ֵ����
**************************************************************************/
void SetRight_Pwm(int moto,u8 mode)
{
	if(mode)
	{
		TIM_SetCompare3(TIM8,TIM8_Period); 
		TIM_SetCompare4(TIM8,TIM8_Period-moto); 
		
		
	}
	else
	{	
		TIM_SetCompare3(TIM8,TIM8_Period-moto); 
		TIM_SetCompare4(TIM8,TIM8_Period); 
	}
}

/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ���
��ڲ�����PWM mode(1Ϊǰ���� 0Ϊ����)
����  ֵ����
**************************************************************************/
void SetThree_Pwm(int moto,u8 mode)
{
	if(mode)
	{
		TIM_SetCompare1(TIM1,TIM8_Period); 
		TIM_SetCompare2(TIM1,TIM8_Period-moto); 
		
	}
	else
	{	
		TIM_SetCompare1(TIM1,TIM8_Period-moto); 
		TIM_SetCompare2(TIM1,TIM8_Period); 
	}
}



/**************************************************************************
�������ܣ�����PWM��ֵ 
��ڲ�������
����  ֵ����
**************************************************************************/
void Xianfu_Pwm(void)
{	
	  //TIM8_Period=1200;    //===PWM������1200 ������1200
	  if(LeftWheel.MotoPwm<-TIM8_Period)  LeftWheel.MotoPwm=-TIM8_Period;	
		if(LeftWheel.MotoPwm>TIM8_Period)   LeftWheel.MotoPwm=TIM8_Period;	

		if(RightWheel.MotoPwm<-TIM8_Period) RightWheel.MotoPwm=-TIM8_Period;	
		if(RightWheel.MotoPwm>TIM8_Period)  RightWheel.MotoPwm=TIM8_Period;	
	
		if(ThreeWheel.MotoPwm<-TIM8_Period) ThreeWheel.MotoPwm=-TIM8_Period;	
		if(ThreeWheel.MotoPwm>TIM8_Period)  ThreeWheel.MotoPwm=TIM8_Period;	
}

/**************************************************************************
�������ܣ�����ֵ����
��ڲ�����int
����  ֵ��unsigned int
**************************************************************************/
int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=0;//temp=-a;  
	  else temp=a;
	  return temp;
}

float Amplitude_PKP=20,Amplitude_PKI=0.1,Amplitude_PKD=25,Amplitude_VKP=2,Amplitude_VKI=3; //PID������ز���
/**************************************************************************
�������ܣ�����PI������
��ڲ���������������ֵ��Ŀ���ٶ�
����  ֵ�����PWM
��������ʽ��ɢPID��ʽ 
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  �Դ����� 
pwm�����������
�����ǵ��ٶȿ��Ʊջ�ϵͳ���棬ֻʹ��PI����
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)
**************************************************************************/
//��PID
float LVelocity_KP=120,LVelocity_KI=3;

int LeftIncremental_PI (int Encoder,int Target)
{ 	
	 static float Bias=0,Pwm=0,Last_bias=0;
	 Bias=Target-Encoder;                                   //����ƫ��
	 Pwm+=LVelocity_KP*(Bias-Last_bias)+LVelocity_KI*Bias;  //����ʽPI������
	
	 if(Target==0){Pwm=0;}																	//Ŀ��Ϊ0ֱ�����0
	 if(Pwm>1200){Pwm=1200;}
	 else if(Pwm<0){Pwm=0;}
	 Last_bias=Bias;	                                     //������һ��ƫ�� 
	 return Pwm;                                           //�������

}

//��PID
float RVelocity_KP=120,RVelocity_KI=3;

int RightIncremental_PI (int Encoder,int Target)
{ 	
	 static float Bias=0,Pwm=0,Last_bias=0;
	 Bias=Target-Encoder;                                  //����ƫ��
	 Pwm+=RVelocity_KP*(Bias-Last_bias)+RVelocity_KI*Bias;   //����ʽPI������
	
	 if(Target==0){Pwm=0;}																//Ŀ��Ϊ0ֱ�����0
	 if(Pwm>1200){Pwm=1200;}
	 else if(Pwm<0){Pwm=0;}
	 Last_bias=Bias;	                                     //������һ��ƫ�� 
	 return Pwm;                                           //�������
}

//��PID
float TVelocity_KP=120,TVelocity_KI=3;

int ThreeIncremental_PI (int Encoder,int Target)
{ 	
	 static float Bias=0,Pwm=0,Last_bias=0;
	 Bias=Target-Encoder;                                  //����ƫ��
	 Pwm+=TVelocity_KP*(Bias-Last_bias)+TVelocity_KI*Bias;   //����ʽPI������
	
	 if(Target==0){Pwm=0;}																	//Ŀ��Ϊ0ֱ�����0
	 if(Pwm>1200){Pwm=1200;}
	 else if(Pwm<0){Pwm=0;}
	 Last_bias=Bias;	                                     //������һ��ƫ�� 
	 return Pwm;                                           //�������
}
/**************************************************************************
�������ܣ�λ��ʽPID������
��ڲ���������������λ����Ϣ��Ŀ��λ��
����  ֵ�����PWM
����λ��ʽ��ɢPID��ʽ 
pwm=Kp*e(k)+Ki*��e(k)+Kd[e��k��-e(k-1)]
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  
��e(k)����e(k)�Լ�֮ǰ��ƫ����ۻ���;����kΪ1,2,,k;
pwm�������
**************************************************************************/
float Position_KP=20,Position_KI=2,Position_KD=0;      //PIDϵ��
int Position_PID (int Encoder,int Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Target-Encoder;                                  //����ƫ��
	 Integral_bias+=Bias;	                                 //���ƫ��Ļ���
	 Pwm=Position_KP*Bias+Position_KI*Integral_bias+Position_KD*(Bias-Last_Bias);       //λ��ʽPID������
	 Last_Bias=Bias;                                       //������һ��ƫ�� 
	 return Pwm;                                           //�������
}


//����ʽPID�㷨
void PID_AbsoluteMode(PID_AbsoluteType* PID)
{
 if(PID->kp      < 0)    PID->kp      = -PID->kp;
 if(PID->ki      < 0)    PID->ki      = -PID->ki;
 if(PID->kd      < 0)    PID->kd      = -PID->kd;
 if(PID->errILim < 0)    PID->errILim = -PID->errILim;

 PID->errP = PID->errNow;  //��ȡ���ڵ�������kp����

 PID->errI += PID->errNow; //�����֣�����ki����

 if(PID->errILim != 0)	   //΢�����޺�����
 {
  if(     PID->errI >  PID->errILim)    PID->errI =  PID->errILim;
  else if(PID->errI < -PID->errILim)    PID->errI = -PID->errILim;
 }
 
 PID->errD = PID->errNow - PID->errOld;//���΢�֣�����kd����

 PID->errOld = PID->errNow;	//�������ڵ����
 
 PID->ctrOut = PID->kp * PID->errP + PID->ki * PID->errI + PID->kd * PID->errD;//�������ʽPID���

}



//ȫ�����˶�����
/*
		Va     					cos@									sin@					L					Vx
		Vb  =	 -cos60cos@+sin60sin@		-cos60sin@-sin60cos@	L			*		Vy
		Vc		 -sin30cos@+cos30sin@		-sin30sin@+cos30cos@	L					 W

		����@ΪС������ϵ����������ϵ�ļн�    
		WΪС������Ľ��ٶ�
		VxΪX���ٶ�
		VyΪY���ٶ�

		VaΪa�����ٶ�
		VbΪb�����ٶ�
		VcΪc�����ٶ�
		LΪ���ӵ����ĵľ���
    ˳ʱ��Ϊ��
*/
#define  L 157 //���ӵ����ĵľ���
void OmniWheelscontrol(s16 Vx,s16 Vy,s16 W,s16 a)
{
	static double Va,Vb,Vc;
	
	Va=Vx*cos(a)+Vy*sin(a)+W*L;
	Vb=Vx*(-cos(PI/3)*cos(a)+sin(PI/6)*sin(a))+Vy*(-cos(PI/3)*sin(a)-sin(PI/3)*cos(a))+W*L;
	Vc=Vx*(-sin(PI/6)*cos(a)+cos(PI/6)*sin(a))+Vy*(-sin(PI/6)*sin(a)+cos(PI/6)*cos(a))+W*L;
	
	LeftWheelSpeedSet ( Va);
	RightWheelSpeedSet(-Vb);
	ThreeWheelSpeedSet(-Vc);
	
	//��ʱ��ת��
	//LeftWheelSpeedSet(-200);
	//RightWheelSpeedSet(200);
	//ThreeWheelSpeedSet(200);
}












