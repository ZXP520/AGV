//���ߣ�������ˮ
#ifndef _FSC_STOS_H_
#define _FSC_STOS_H_

/****************************�������stm32f1xx,���Ϊ��ӦоƬ��ͷ�ļ�*******************************************/
#include "stm32f10x.h"
//#include "stm32f4xx.h"
/***************************************************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>          
/****************************************�û����Զ���***********************************************************/

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#define OS_CLOCK_TIME       1000     //����ʱ����Ƭ,ÿ������������е�ʱ�䣬��λ:΢�� us  
#define OS_MAX_TASKS        16       //������=�û�������+2  ������:3-255  �û�����ʵ����Ҫ�����������޸�
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#define TIMER_SIZE          8       //ϵͳ���ⶨʱ������(����ֵ) 
#define FLAG_SIZE           8       //��־����(����ֵ) 
#define MUTEX_SIZE          8       //��������(����ֵ) 
#define MBOX_SIZE           8       //��������(����ֵ) 
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/***************************************************************************************************************/

#define TASK_NAME_SIZE          32  //������       �ַ���󳤶�
#define OS_PERIP_USART_BUFF_LEN 32  //ϵͳָ��     �ַ���󳤶�
#define OS_CMD_LEVEL            8   //ϵͳָ�����

#define TaskIDLE_StkSize        16  //���������ջ��С
#define TaskManage_StkSize      128 //��������������ջ��С

#define TASK_CREATING    0           //����̬
#define TASK_RUNNING     1           //����̬
#define TASK_PAUSING     2           //��̬ͣ
#define TASK_BACKRUNNING 3           //��̨����̬
#define TASK_DELETING    4           //ɾ��̬
                                   
#define OS_FALSE 0                  //��
#define OS_TRUE  1                  //��

typedef unsigned char  INT8U;            
typedef unsigned short INT16U;           
typedef unsigned int   INT32U;           
typedef unsigned int   OS_STK;

typedef struct
{
	INT32U Timer[TIMER_SIZE];      //���ⶨʱ��
	INT8U  FLAG[FLAG_SIZE];        //��־��
  INT8U  MUTEX[MUTEX_SIZE];      //������
	INT32U *MBOX[MBOX_SIZE];       //����
}OS_SYS;
extern OS_SYS OS_Sys;            //ϵͳ���ܽṹ��(���û�ʹ��)

typedef struct
{
	INT8U  OS_USART_COUNT;
	char   OS_USART_RX_BUFF[OS_PERIP_USART_BUFF_LEN];	
}OS_PERIP;
extern OS_PERIP OS_Perip;        //ϵͳ����ָ�����(�û�������)

typedef struct
{
	INT8U  OSRunning;
	INT8U  OSIntNesting; 
	INT8U  OSLockNesting; 
	INT32U OSCtxSwCtr; 
	INT16U OSTaskNext;
	INT8U  OSTaskNextRunFlag;
	INT8U  OSTaskSwitchBack;
	INT32U TaskTickLeft;
	INT32U TaskTimeSliceCount; 
}OS_SYSTEM;
extern OS_SYSTEM OS_System; 

typedef struct 
{
	OS_STK    *OSTCBStkPtr;  
  INT32U    OSTCBTaskAdd;	
	INT16U    OSTCBTaskNum; 
	char      OSTCBTaskStr[TASK_NAME_SIZE];
	INT8U     OSTCBStat;
	INT32U    OSTCBTaskTSCnt;     
  INT32U    OSTCBTaskCPUOccRate;  
} OS_TCB; 
extern OS_TCB OSTCBTbl[OS_MAX_TASKS];  
extern OS_TCB *OSTCBRun; 
extern OS_TCB *OSTCBCur; 
extern OS_TCB *OSTCBNext;

/***************************************�˴���ע�͵ĺ����û����������****************************************/                          
void OSStart(void); 
void OSInit(void);
void OSCtxSw(void); 
void OSTaskCreate(char* taskstr,void (*task),OS_STK *p_tos,INT8U taskstate); //���񴴽�����(����main������ʹ��)
INT8U OSTaskStateGet(void* Taskx); //��ȡ����״̬
void OSTaskStateSet(void* Taskx,INT8U TaskState); //����״̬����,����1��������� ����2������״̬
void OSTaskSwitchBack(void* Taskx); //������ת  ������
void OSTaskSwitch(void* Taskx); //������ת  ��������
void OSSchedLock(void);                      //�����л���������(�еĴ������һ������ɣ���;���ܱ��л���ȥ,���봦�����������OSSchedUnlock()���)  
void OSSchedUnlock(void);                    //�����л���������
void OSIntEnter(void);					             //�����ж�ʱ����  
void OSIntExit(void);	                       //�˳��ж�ʱ����    
void OSFlagPost(uint16_t FNum);              //���ͱ�־��
void OSFlagPend(uint16_t FNum);              //�ȴ���־��
void OSMutexPost(uint16_t MNum);             //���ͻ�����
void OSMutexPend(uint16_t MNum);             //�ȴ�������
void OSMboxPost(uint16_t MNum,void* fp);     //�����ʼ�(��ַ)
void* OSMboxPend(uint16_t MNum);             //�ȴ��ʼ�
INT32U GetOSSliceTime(void);                 //��ȡOS������
INT32U GetOSTimerVal(void);                  //��ȡϵͳ��ʱ����ǰ����ֵ(Ĭ��Ϊ9MHz��ÿ�μ�����ʱ1/9 us)
void delay_us(INT32U nus);                   //΢����ʱ����
void delay_ms(INT32U nms);	                 //������ʱ����
void OS_delayMs(volatile INT32U nms);        //������ʱ����
/****************************************************************************************************************/

#endif

