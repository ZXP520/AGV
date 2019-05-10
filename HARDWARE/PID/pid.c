#include "pid.h"
#include <stdio.h> 
#include <stdlib.h> 
#include "include.h"


struct 
{
	float kp;
	float ki;
	float kd;
	float error[3];
	u16 PWM;
}pid;
///PID初始化
void PID_Init(void)
{
	pid.kp=5;
  pid.ki=0;
	pid.kd=0;
	pid.PWM=0;
	pid.error[0]=0;
	pid.error[1]=0;
	pid.error[2]=0;
}


///PID计算程序
// set  设定值
// yout 实际值
//返回值为给的PWM
u16 PID_Calculation(u16 set,u16 yout)
{
	u16 out=0;
	pid.error[0]=set-yout;
	out=pid.kp*(pid.error[0]-pid.error[1])  +						    //p
	pid.ki* pid.error[0] 							  + 		//i
	pid.kd*(pid.error[0]-2*pid.error[1] + pid.error[2]);		//d
	
	pid.PWM+=out;
	pid.error[1]=pid.error[0];
	pid.error[2]=pid.error[2];
	
	printf("%d\n",pid.PWM);
	return pid.PWM;
}









