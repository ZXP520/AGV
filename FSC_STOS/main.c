/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
1.  ϵͳ�ṩ��ȷС��ʱ����:
    delay_us(); ��������������֮���΢�뼶��ʱ,��λ:us΢�� (ϵͳ�����л�ֱ����ʱ���)
	  delay_ms(); ��������������֮��ĺ��뼶��ʱ,��λ:ms���� (ϵͳ�����л�ֱ����ʱ���)
    ����ʱ����
    OS_delayMs(); ������������������֮�����ʱ,��λ:ms����  �����ú������л�����������ֱ����ʱ�䵽������ϵͳ�л�������
		
		��Ҫ������c�ļ���ʹ����ʱ��������������������c�ļ��Ŀ�ͷ��,����c�ļ������FSC_STOS.hͷ�ļ�
		void delay_us(unsigned int nus);
		void delay_ms(unsigned int nms);		

2.�������
	*��Ӳ��裺(�����Task13Ϊ��,�����������⣬ֻҪ��Ӧ����������һ�¼��ɡ�����Ϊ����ȡ��ΪTask13)
	*1.void Task13(void);//����13���� 
  *2.#define Task13_StkSize 128       //����13��ջ��С
  *3.OS_STK Task13_Stk[Task13_StkSize];//����13��ջ
	*4.OSTaskCreate("Task13",Task13,(OS_STK*)&Task13_Stk[Task13_StkSize-1],TASK_RUNNING);//��main()�����ﴴ�������񣬶�ջ��С>=64,�Ӿ��������ڴ������
	*5.void Task13(void) //����13
		{
			 while(1) 
			 {		
				 //printf("Task13 is running\r\n");//ʾ�����룬ʹ��ʱɾ��	
				 //OS_delayMs(1000); 				       //ʾ�����룬ʹ��ʱɾ��	
			 }
		}

************************************************/

//(˫��fsc_stos���Ҽ�->Open document "fsc_stos.h"���ɴ򿪣�����ͬ�� )
#include "fsc_stos.h"  //ʹ�ö������ں�

#include "sys.h"
#include "timer.h"
#include "include.h"
#include "led.h"
#include "runcontrol.h"
#include "control.h"
#include "gy85.h"
#include "kalman.h"
#include "dealdata.h"
#include "stm32f10x_it.h" 
#include "Encoder.h"
#include "bsp_usart.h"


/******************************�����������*************************/
void Task1(void); //����1����
void Task2(void); //����2����
void Task3(void); //����3����
void Task4(void); //����4����
void Task5(void); //����5����

#define Task1_StkSize 128       //����1��ջ��С����С���⣩
#define Task2_StkSize 128       //����2��ջ��С (��ͬ�����С���Բ�һ��)
#define Task3_StkSize 128       //����3��ջ��С (Ӧ���ݾ��������С����)
#define Task4_StkSize 128       //����4��ջ��С������ú����й����в��ɸı䣩
#define Task5_StkSize 128       //����5��ջ��С

__align(8) OS_STK Task1_Stk[Task1_StkSize]; //����1��ջ
__align(8) OS_STK Task2_Stk[Task2_StkSize]; //����2��ջ
__align(8) OS_STK Task3_Stk[Task3_StkSize]; //����3��ջ
__align(8) OS_STK Task4_Stk[Task4_StkSize]; //����4��ջ
__align(8) OS_STK Task5_Stk[Task5_StkSize]; //����5��ջ
/*******************************************************************/
/***************************************�û��������񲢼����ں�����***************************************/
int main(void)
{     
	  //ȫ�ֳ�ʹ��,�Ƽ�����������ʹ�õ��ĳ�ʹ�����ڴ˴���task�����õ��ĳ�ʹ������task��
    /************************************************************************************/	
	  /*------------------ȫ�ֳ�ʹ����-------------------*/
	  NVIC_Configuration(); 	 			  
	  USART2_Config(115200);
		USART3_Config(115200);
	  Time_Config();							
	  InitGY85();
		Init_PID();
	
		//LeftWheelSpeedSet(200);
		//RightWheelSpeedSet(200);
		//ThreeWheelSpeedSet(300);
		//FourWheelSpeedSet(300);
		//SetLeft_Pwm(0,1);
    //SetRight_Pwm(400,1);
    //SetThree_Pwm(400,1);
    //SetFour_Pwm(400,1);
	  //OmniWheelscontrol(0,0,1,0);

	  
	
	  /************************************************************************************/	
    OSInit(); //ϵͳ��ʹ��
	  /********************************��ϵͳ�д�������***********************************/
    OSTaskCreate("Task1",Task1,(OS_STK*)&Task1_Stk[Task1_StkSize-1],TASK_RUNNING); //��������1
    OSTaskCreate("Task2",Task2,(OS_STK*)&Task2_Stk[Task2_StkSize-1],TASK_RUNNING); //��������2
    OSTaskCreate("Task3",Task3,(OS_STK*)&Task3_Stk[Task3_StkSize-1],TASK_RUNNING); //��������3
	  OSTaskCreate("Task4",Task4,(OS_STK*)&Task4_Stk[Task4_StkSize-1],TASK_RUNNING); //��������4
		OSTaskCreate("Task5",Task5,(OS_STK*)&Task5_Stk[Task5_StkSize-1],TASK_RUNNING); //��������5
	  /***********************************************************************************/
    OSStart();//OS��ʼ����
}
/***********************************************************************************/
/*��void taskX(void)������ͨ��int main(void)ʹ�ü���,�����൱�ڶ��main()����������*/

/****************************************�û�ȫ�ֱ������궨����*****************************************/

/*******************************************************************************************************/
/*********************************�û�����ʵ�������************************************/
void Task1(void)  //����1  �õ����������� 
{ 	
	while(1) 
	 {
		 //PID����Ӧ�÷ŵ��ж��е���	  
		 Get_Encoder();
  	 OS_delayMs(10);			//ʾ�����룬ʹ��ʱɾ��		 
	 }	
}

void Task2(void) //����2   PID����
{
	while(1) 
	 {
		 //PID����ʹ����
			#if PID_ENABLE==1 
			OSSchedLock();         //�����л����� 
			RunWheelcontrol();
		  OSSchedUnlock();
			#endif	  
  	  OS_delayMs(10);			//ʾ�����룬ʹ��ʱɾ��		 
	 }			
}

void Task3(void) //����3 ����2�������ݸ�ROS
{	
	while(1) 
	 {	 
		 SendEncoderAndIMU20Ms();
  	 OS_delayMs(5);				 
	 }			
}

void Task4(void) //����4  ��ӡ����
{
	while(1) 
	 {		
		 u3_printf("L:%d  :%d	R:%d  :%d	T:%d	:%d	F:%d	:%d\n",
		 LeftWheel.NowSpeed,LeftWheel.AimSpeed,RightWheel.NowSpeed,RightWheel.AimSpeed,
		 ThreeWheel.NowSpeed,ThreeWheel.AimSpeed,FourWheel.NowSpeed,FourWheel.AimSpeed);
  		OS_delayMs(500); 			//ʾ�����룬ʹ��ʱɾ��		
	 }
}

void Task5(void) //����5   200MS����Ƿ������ݣ�û��������ֹͣ�˶�
{
	static u8 Time_Cnt=0;
	while(1) 
	 {	
		 if(DealData_Rx.Success_Flag)
		 {
			 Time_Cnt=0;
			 DealData_Rx.Success_Flag=0;
		 }
		 else
		 {
			 Time_Cnt++;
		 }
		 if(Time_Cnt>200)
		 {
			 //LeftWheelSpeedSet (0);
			 //RightWheelSpeedSet(0);
			 //ThreeWheelSpeedSet(0);
			 //FourWheelSpeedSet(0);
			 //AllWheel.stop_flag=1;
		 }
     OS_delayMs(1); 				//1Ms��һ��
	 }
}

/********************************************************************************************/

