#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"
  /**************************************************************************

**************************************************************************/
#define VERSIONNUMBER   0x0100  //�汾1.00
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#define PI 3.14159265
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#if		VERSION==0
	#define Wheel_D          	67   //mm����ֱ��
	#define Wheel_SPACING		  263  //mm�ּ��
#elif VERSION==1
	#define Wheel_D          	59   //mm����ֱ��
	#define Wheel_SPACING		  157  //mm�ּ��
#endif
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#define Wheel_RATIO     	56   //���ٱ�
#define ENCODER_LINE     	11   //����������
#define SPEED_TO_ENCODER  (float)(4*Wheel_RATIO*ENCODER_LINE/(Wheel_D*PI*100))				//�ٶ�ת��������  (��/10ms)(4Ϊ������ģʽ��һ�������ĸ�����)(������Ӧ�������Ÿĳɳ��ԣ���Ȼ���0)
#define TIM8_Period  1200			 //TIME8��װֵ
#define MAXSPEED     500       //����ٶ�mm/s
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
typedef struct
{
  _Bool 	Direct;		//����
  int 	AimsEncoder;//Ŀ��������
	int 	MotoPwm;		//����PWM
	s16   NowSpeed;		//���ӵ�ǰ�ٶ�
	s16   AimSpeed;   //����Ŀ���ٶ�
	
	//�������ӵ�״̬AllWheel
	//u8    stop_flag;  //ֹͣ��־
	union
	{
			struct bit_feild
			{
					char bit0: 1;  //��ͣ��־
					char bit1: 1;  //ǰ����
					char bit2: 1;  //ǰ����
					char bit3: 1;  //����
					char bit4: 1;  //����
					char bit5: 1;  //������
					char bit6: 1;  //���ٶȼ�
					char bit7: 1;  //������
					char bit8: 1;  //��ص�ѹ
					char bit9: 1;
					char bit10:1;
					char bit11:1;
					char bit12:1;
					char bit13:1;
					char bit14:1;
					char bit15:1;
			} bits;
			s16 data;
	}Erroe_flag;
	u8    Electricity;    //����
	
}Wheel;
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/*����ʽPID�㷨���ӿڲ����ṹ����*/
typedef struct
{
	/*PID�㷨�ӿڱ��������ڸ��û���ȡ���޸�PID�㷨������*/
 float kp;     //����ϵ��
 float ki;     //����ϵ��
 float kd;     //΢��ϵ��
 float errILim;//����������
 
 float errNow;//��ǰ�����
 float errLast;//�ϴε����
 float ctrOut;//���������
	
}PID_AddType;
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/*����ʽPID�㷨���ӿڲ����ṹ����*/
typedef struct 
{
 /*PID�㷨�ӿڱ��������ڸ��û���ȡ���޸�PID�㷨������*/
 float kp;     //����ϵ��
 float ki;     //����ϵ��
 float kd;     //΢��ϵ��
 float errILim;//����������
 
 float errNow;//��ǰ�����
 float ctrOut;//���������
 
 /*PID�㷨�ڲ���������ֵ�����޸�*/
 float errOld;
 float errP;
 float errI;
 float errD;
 
}PID_AbsoluteType;
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
extern Wheel LeftWheel,RightWheel,ThreeWheel,FourWheel,AllWheel;//���������ֽṹ��
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void Init_PID(void);
void PID_AbsoluteMode(PID_AbsoluteType* PID);
void RunWheelcontrol(void);
void SetLeft_Pwm(int moto,u8 mode);
void SetRight_Pwm(int moto,u8 mode);
void SetThree_Pwm(int moto,u8 mode);
void SetFour_Pwm(int moto,u8 mode);
void Xianfu_Pwm(void);
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
static int myabs(int a);
static int LeftIncremental_PI (int Encoder,int Target);
static int RightIncremental_PI (int Encoder,int Target);
static int ThreeIncremental_PI (int Encoder,int Target);
static int FourIncremental_PI (int Encoder,int Target);
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
//�����ٶ�����
void LeftWheelSpeedSet(int speed);
//�����ٶ�����
void  RightWheelSpeedSet(int speed);
//�����ٶ�����
void  ThreeWheelSpeedSet(int speed);
//�����ٶ�����
void  FourWheelSpeedSet(int speed);
//����ȫ�����˶�����
void OmniWheelscontrol(s16 Vx,s16 Vy,s16 W,s16 a);
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#endif
