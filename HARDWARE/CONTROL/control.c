#include "control.h"		
#include "sys.h"
#include "include.h"
#include "led.h"
#include "timer.h"
#include "usart.h"
#include "Encoder.h"
/**************************************************************************
�������ܣ������ٶ����� mm/s
��ڲ�����direction(1Ϊǰ����0Ϊ����)
					speed     ���ٶ�mm/s��34mm/s-376mm/s  ����Ϊ34mm/s
	
**************************************************************************/
Wheel LeftWheel,RightWheel,AllWheel;//���������ֽṹ��


//�����ٶ�����
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

//�����ٶ�����
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


/**************************************************************************
�������ܣ�PID�˶�����
					10ms��һ��
					��ϵͳ�ĵδ�ʱ���е���
**************************************************************************/
PID_AbsoluteType PID_Control;//����PID�㷨�Ľṹ��


void RunWheelcontrol(void)
{	
	static u8 cnt=0;
	float temp=0;
	static float speed_usart=0;
	cnt++;
	
	//ͣ����־ ���뱧��״̬
	if(AllWheel.stop_flag)
	{
		TIM_SetCompare1(TIM8,TIM8_Period); 
		TIM_SetCompare2(TIM8,TIM8_Period);
		TIM_SetCompare3(TIM8,TIM8_Period); 
		TIM_SetCompare4(TIM8,TIM8_Period); 
		return;
	}
	
	//���PID���ٺ��PWM
	LeftWheel.MotoPwm =myabs( LeftIncremental_PI(abs(GetEncoder.V5) ,LeftWheel.AimsEncoder ));//���PID���ٺ��PWM
	RightWheel.MotoPwm=myabs(RightIncremental_PI(abs(GetEncoder.V3) ,RightWheel.AimsEncoder));
	
	Xianfu_Pwm();//�޷�
	
	if(cnt%10==0)
	{
		//u2_printf("PWM:	%d   Right:	%d	PWM:	%d  Left:	%d	aim:%d\r\n",RightWheel.MotoPwm,abs(GetEncoder.V3),LeftWheel.MotoPwm,abs(GetEncoder.V5),LeftWheel.AimsEncoder);
		temp=GetEncoder.V3;
		printf("@%d@",(int)(temp/0.11));
	}
	
	//����PWM�뷽��
	SetLeft_Pwm (LeftWheel.MotoPwm  ,LeftWheel.Direct );
	SetRight_Pwm(RightWheel.MotoPwm ,RightWheel.Direct);
	
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
float LVelocity_KP=120,LVelocity_KI=5;

int LeftIncremental_PI (int Encoder,int Target)
{ 	
	 static float Bias=0,Pwm=0,Last_bias=0;
	 Bias=Target-Encoder;                                  //����ƫ��
	 Pwm+=LVelocity_KP*(Bias-Last_bias)+LVelocity_KI*Bias;   //����ʽPI������
	 if(Pwm>1200){Pwm=1200;}
	 else if(Pwm<0){Pwm=0;}
	 Last_bias=Bias;	                                     //������һ��ƫ�� 
	 return Pwm;                                           //�������

}

//��PID
float RVelocity_KP=120,RVelocity_KI=5;

int RightIncremental_PI (int Encoder,int Target)
{ 	
	 static float Bias=0,Pwm=0,Last_bias=0;
	 Bias=Target-Encoder;                                  //����ƫ��
	 Pwm+=RVelocity_KP*(Bias-Last_bias)+RVelocity_KI*Bias;   //����ʽPI������
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
