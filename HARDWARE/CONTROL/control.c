#include "control.h"		
#include "sys.h"
#include "include.h"
#include "led.h"
#include "timer.h"
#include "usart.h"
#include <math.h>
#include "Encoder.h"


Wheel LeftWheel,RightWheel,ThreeWheel,FourWheel,AllWheel;//���������ֽṹ��

/*******************************************************************************
* Function Name  : LeftWheelSpeedSet
* Description    : �����ٶ�����
* Input          : �����ٶ� 
* Output         : None
* Return         : None 
****************************************************************************** */
void LeftWheelSpeedSet(int speed)
{
//���ַ��� ����
#if	VERSION==0
	  speed=-speed;
#endif
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
	LeftWheel.AimSpeed=speed;   //Ŀ���ٶ�
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
	RightWheel.AimSpeed=speed;
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
	ThreeWheel.AimSpeed=speed;
	ThreeWheel.AimsEncoder=speed*SPEED_TO_ENCODER+0.5;//+0.5��������
}


/*******************************************************************************
* Function Name  : LeftWheelSpeedSet
* Description    : �����ٶ�����
* Input          : �����ٶ� 
* Output         : None
* Return         : None 
****************************************************************************** */
void  FourWheelSpeedSet(int speed)
{
	if(speed>=0)//������
	{
		if(speed>MAXSPEED){speed=MAXSPEED;}//����
		FourWheel.Direct=1;
	}
	else        //������
	{
		speed=-speed;
		if(speed>MAXSPEED){speed=MAXSPEED;}//����
		FourWheel.Direct=0;
	}
	FourWheel.AimSpeed=speed;
	FourWheel.AimsEncoder=speed*SPEED_TO_ENCODER+0.5;//+0.5��������
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
	LeftWheel.MotoPwm =myabs( LeftIncremental_PI(abs(GetEncoder.V3) ,LeftWheel.AimsEncoder ));//���PID���ٺ��PWM
	RightWheel.MotoPwm=myabs(RightIncremental_PI(abs(GetEncoder.V1) ,RightWheel.AimsEncoder));
	ThreeWheel.MotoPwm=myabs(ThreeIncremental_PI(abs(GetEncoder.V5) ,ThreeWheel.AimsEncoder));
	FourWheel.MotoPwm =myabs( FourIncremental_PI(abs(GetEncoder.V2) ,ThreeWheel.AimsEncoder));
	
	Xianfu_Pwm();//�޷�

	//����PWM�뷽��
	SetLeft_Pwm (LeftWheel.MotoPwm  ,LeftWheel.Direct );
	SetRight_Pwm(RightWheel.MotoPwm ,RightWheel.Direct);
	SetThree_Pwm(ThreeWheel.MotoPwm ,ThreeWheel.Direct);
	SetFour_Pwm (FourWheel.MotoPwm  ,FourWheel.Direct );
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
		TIM_SetCompare1(TIM8,TIM8_Period); 
		TIM_SetCompare2(TIM8,TIM8_Period-moto); 
	}
	else
	{
		TIM_SetCompare1(TIM8,TIM8_Period-moto); 
		TIM_SetCompare2(TIM8,TIM8_Period); 
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
		TIM_SetCompare1(TIM4,TIM8_Period); 
		TIM_SetCompare2(TIM4,TIM8_Period-moto); 
		
	}
	else
	{	
		TIM_SetCompare1(TIM4,TIM8_Period-moto); 
		TIM_SetCompare2(TIM4,TIM8_Period); 
	}
}

/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ���
��ڲ�����PWM mode(1Ϊǰ���� 0Ϊ����)
����  ֵ����
**************************************************************************/
void SetFour_Pwm(int moto,u8 mode)
{
	if(mode)
	{
		TIM_SetCompare3(TIM4,TIM8_Period); 
		TIM_SetCompare4(TIM4,TIM8_Period-moto); 
	}
	else
	{	
		TIM_SetCompare3(TIM4,TIM8_Period-moto); 
		TIM_SetCompare4(TIM4,TIM8_Period); 
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
	
		if(FourWheel.MotoPwm<-TIM8_Period) FourWheel.MotoPwm=-TIM8_Period;	
		if(FourWheel.MotoPwm>TIM8_Period)  FourWheel.MotoPwm=TIM8_Period;	
}

/**************************************************************************
�������ܣ�����ֵ����
��ڲ�����int
����  ֵ��unsigned int
**************************************************************************/
static int myabs(int a)
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

PID_AddType LeftPID,RightPID,ThreePID,FourPID;

//PID������ʼ��
void Init_PID(void)
{
	LeftPID.kp=120;
	LeftPID.ki=3;
	
	RightPID.kp=120;
	RightPID.ki=3;
	
	ThreePID.kp=120;
	ThreePID.ki=3;
	
	FourPID.kp=120;
	FourPID.ki=3;
}

//��PID

static int LeftIncremental_PI (int Encoder,int Target)
{ 	
	LeftPID.errNow=Target-Encoder;  																												//����ƫ��
	LeftPID.ctrOut+=LeftPID.kp*(LeftPID.errNow-LeftPID.errLast)+LeftPID.ki*LeftPID.errNow;	//����ʽPI������
	LeftPID.errLast=LeftPID.errNow;																													//������һ��ƫ�� 
	if(Target==0){LeftPID.ctrOut=0;}
	if(LeftPID.ctrOut>TIM8_Period){LeftPID.ctrOut=TIM8_Period;}
	else if(LeftPID.ctrOut<0){LeftPID.ctrOut=0;}
	return LeftPID.ctrOut;																																	//�������
}

//��PID
static int RightIncremental_PI (int Encoder,int Target)
{ 	
	RightPID.errNow=Target-Encoder;  																												//����ƫ��
	RightPID.ctrOut+=RightPID.kp*(RightPID.errNow-RightPID.errLast)+RightPID.ki*RightPID.errNow;	//����ʽPI������
	RightPID.errLast=RightPID.errNow;																													//������һ��ƫ�� 
	if(Target==0){RightPID.ctrOut=0;}
	if(RightPID.ctrOut>TIM8_Period){RightPID.ctrOut=TIM8_Period;}
	else if(RightPID.ctrOut<0){RightPID.ctrOut=0;}
	return RightPID.ctrOut;																																	//�������
}

//��PID
static int ThreeIncremental_PI (int Encoder,int Target)
{ 	
	ThreePID.errNow=Target-Encoder;  																												//����ƫ��
	ThreePID.ctrOut+=ThreePID.kp*(ThreePID.errNow-ThreePID.errLast)+ThreePID.ki*ThreePID.errNow;	//����ʽPI������
	ThreePID.errLast=ThreePID.errNow;																													//������һ��ƫ�� 
	if(Target==0){ThreePID.ctrOut=0;}
	if(ThreePID.ctrOut>TIM8_Period){ThreePID.ctrOut=TIM8_Period;}
	else if(ThreePID.ctrOut<0){ThreePID.ctrOut=0;}
	return ThreePID.ctrOut;																																	//�������
}

//��PID
static int FourIncremental_PI (int Encoder,int Target)
{ 	

	FourPID.errNow=Target-Encoder;  																												//����ƫ��
	FourPID.ctrOut+=FourPID.kp*(FourPID.errNow-FourPID.errLast)+FourPID.ki*FourPID.errNow;	//����ʽPI������
	FourPID.errLast=FourPID.errNow;																													//������һ��ƫ�� 
	if(Target==0){FourPID.ctrOut=0;}
	if(FourPID.ctrOut>TIM8_Period){FourPID.ctrOut=TIM8_Period;}
	else if(FourPID.ctrOut<0){FourPID.ctrOut=0;}
	return FourPID.ctrOut;																																	//�������
	
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
//��ת˳ʱ��Ϊ��
void OmniWheelscontrol(s16 Vx,s16 Vy,s16 W,s16 a)
{
	static double Va,Vb,Vc;
	Vx=-Vx;
	W=-W;
	Va=Vx*cos(a)+Vy*sin(a)+W*L;
	Vb=Vx*(-cos(PI/3)*cos(a)+sin(PI/6)*sin(a))+Vy*(-cos(PI/3)*sin(a)-sin(PI/3)*cos(a))+W*L;
	Vc=Vx*(-sin(PI/6)*cos(a)+cos(PI/6)*sin(a))+Vy*(-sin(PI/6)*sin(a)+cos(PI/6)*cos(a))+W*L;
	
	LeftWheelSpeedSet ( Va);
	RightWheelSpeedSet( Vb);
	ThreeWheelSpeedSet( Vc);
	
	//��ʱ��ת��
	//LeftWheelSpeedSet( 200);
	//RightWheelSpeedSet(200);
	//ThreeWheelSpeedSet(200);
}












