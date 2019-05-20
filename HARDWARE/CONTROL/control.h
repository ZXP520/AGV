#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
#define PI 3.14159265
#define Wheel_D          	67   //mm轮子直径
#define Wheel_SPACING		  263  //mm轮间距
#define Wheel_RATIO     	56   //减速比
#define ENCODER_LINE     	11   //编码器线数
#define SPEED_TO_ENCODER  4*Wheel_RATIO*ENCODER_LINE/Wheel_D/PI/100				//速度转编码脉冲  (个/10ms)(4为编码器模式，一个脉冲四个计数)
#define TIM8_Period  1200			 //TIME8重装值
#define MAXSPEED     500       //最大速度mm/s

typedef struct
{
  _Bool 	Direct;			//方向
  int 	AimsEncoder;//目标脉冲数
	int 	MotoPwm;		//轮子PWM
	int   speed;			//轮子速度
	
	//整个轮子的状态AllWheel
	u8    stop_flag;  //停止标志
	u8    imu_num;		//陀螺仪数据量
	u8    navigation_flag;//导航标志
	
}Wheel;


/*绝对式PID算法，接口参数结构类型*/
typedef struct 
{
 /*PID算法接口变量，用于给用户获取或修改PID算法的特性*/
 float kp;     //比例系数
 float ki;     //积分系数
 float kd;     //微分系数
 float errILim;//误差积分上限
 
 float errNow;//当前的误差
 float ctrOut;//控制量输出
 
 /*PID算法内部变量，其值不能修改*/
 float errOld;
 float errP;
 float errI;
 float errD;
 
}PID_AbsoluteType;

void PID_AbsoluteMode(PID_AbsoluteType* PID);


extern Wheel LeftWheel,RightWheel,ThreeWheel,AllWheel;//定义左右轮结构体

void RunWheelcontrol(void);
void SetLeft_Pwm(int moto,u8 mode);
void SetRight_Pwm(int moto,u8 mode);
void SetThree_Pwm(int moto,u8 mode);
void Xianfu_Pwm(void);
int myabs(int a);
int LeftIncremental_PI (int Encoder,int Target);
int RightIncremental_PI (int Encoder,int Target);
int ThreeIncremental_PI (int Encoder,int Target);
//左轮速度设置
void LeftWheelSpeedSet(int speed);
//右轮速度设置
void  RightWheelSpeedSet(int speed);
//三轮速度设置
void  ThreeWheelSpeedSet(int speed);
//三轮全向轮运动控制
void OmniWheelscontrol(u8 Vx,u8 Vy,u8 W,u8 a);
#endif
