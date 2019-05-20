#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
#define PI 3.14159265
#define Wheel_D          	67   //mm����ֱ��
#define Wheel_SPACING		  263  //mm�ּ��
#define Wheel_RATIO     	56   //���ٱ�
#define ENCODER_LINE     	11   //����������
#define SPEED_TO_ENCODER  4*Wheel_RATIO*ENCODER_LINE/Wheel_D/PI/100				//�ٶ�ת��������  (��/10ms)(4Ϊ������ģʽ��һ�������ĸ�����)
#define TIM8_Period  1200			 //TIME8��װֵ
#define MAXSPEED     500       //����ٶ�mm/s

typedef struct
{
  _Bool 	Direct;			//����
  int 	AimsEncoder;//Ŀ��������
	int 	MotoPwm;		//����PWM
	int   speed;			//�����ٶ�
	
	//�������ӵ�״̬AllWheel
	u8    stop_flag;  //ֹͣ��־
	u8    imu_num;		//������������
	u8    navigation_flag;//������־
	
}Wheel;


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

void PID_AbsoluteMode(PID_AbsoluteType* PID);


extern Wheel LeftWheel,RightWheel,ThreeWheel,AllWheel;//���������ֽṹ��

void RunWheelcontrol(void);
void SetLeft_Pwm(int moto,u8 mode);
void SetRight_Pwm(int moto,u8 mode);
void SetThree_Pwm(int moto,u8 mode);
void Xianfu_Pwm(void);
int myabs(int a);
int LeftIncremental_PI (int Encoder,int Target);
int RightIncremental_PI (int Encoder,int Target);
int ThreeIncremental_PI (int Encoder,int Target);
//�����ٶ�����
void LeftWheelSpeedSet(int speed);
//�����ٶ�����
void  RightWheelSpeedSet(int speed);
//�����ٶ�����
void  ThreeWheelSpeedSet(int speed);
//����ȫ�����˶�����
void OmniWheelscontrol(u8 Vx,u8 Vy,u8 W,u8 a);
#endif
