#include "fsc_stos.h" 

void OS_INT_OFF(void); //�ر����ж�           
void OS_INT_ON(void);  //�����ж�
OS_STK* OSTaskStkInit(void (*task),OS_STK *p_tos);
void SysTickInit(INT16U Nms);
void OS_TaskIdle(void); 
void OS_CreatTaskIdle(void);
void OS_TaskManage(void);
void OSCreatTaskManage(void);
extern void OSCoreSW(INT16U OSTaskMax,INT16U OSTimerSize);

__align(8) OS_STK TASK_IDLE_STK[TaskIDLE_StkSize];    //���������ջ
__align(8) OS_STK TASK_MANAGE_STK[TaskManage_StkSize];//��������������ջ
char * Systick_priority = (char *)0xe000ed23;
INT32U Null='\0';
INT8U MutexOSTaskNum[MUTEX_SIZE];

OS_SYS OS_Sys;
OS_PERIP OS_Perip;
OS_SYSTEM OS_System;
OS_TCB OSTCBTbl[OS_MAX_TASKS];    
OS_TCB *OSTCBRun; 
OS_TCB *OSTCBCur; 
OS_TCB *OSTCBNext;

#define CMD_LEVEL_SIZE 5 //CMDָ��Ƕ�ײ���

/**********************��һ��************************/
#define CMD_LEVEL_1_OPEN  1
#define CMD_LEVEL_1_CLOSE 2
#define CMD_LEVEL_1_TaskManage 3
/****************************************************/
/**********************�ڶ���************************/
#define CMD_LEVEL_2_TaskManage 1
#define CMD_LEVEL_2_Task       2
/****************************************************/


struct CMD
{
  INT8U Level[CMD_LEVEL_SIZE];
	INT16U StrncmpPos;
}cmd;	


OS_STK* OSTaskStkInit(void (*task),OS_STK *p_tos)
{
	  //ֻҪ�����ж�,��ת�жϴ�����ǰ xPSR,PC,LR,R12,R3-R0���Զ����浽оƬ��ջ�ڴ���(�˲�����оƬ�Զ���ɵ�)����R4-R11��Ҫ�ֶ����浽�����ڴ�
    OS_STK *stk;
    stk = p_tos;
    *(stk)    = (INT32U)0x01000000L;   // xPSR                                               
    *(--stk)  = (INT32U)task;          // Entry Point   �ռ䱣����������ڵ�ַ             
    *(--stk)  = (INT32U)0xFFFFFFFEL;   // R14 (LR)         
    *(--stk)  = (INT32U)0x12121212L;   // R12                                                   
    *(--stk)  = (INT32U)0x03030303L;   // R3                                                   
    *(--stk)  = (INT32U)0x02020202L;   // R2                                                    
    *(--stk)  = (INT32U)0x01010101L;   // R1                                                     
    *(--stk)  = (INT32U)0;             // R0 : argument   	
    *(--stk)  = (INT32U)0x11111111L;   // R11          
    *(--stk)  = (INT32U)0x10101010L;   // R10         
    *(--stk)  = (INT32U)0x09090909L;   // R9         
    *(--stk)  = (INT32U)0x08080808L;   // R8         
    *(--stk)  = (INT32U)0x07070707L;   // R7         
    *(--stk)  = (INT32U)0x06060606L;   // R6         
    *(--stk)  = (INT32U)0x05050505L;   // R5          
    *(--stk)  = (INT32U)0x04040404L;   // R4     
	  //���ϲ����ǿɶ��Կɸ�Ϊ :  stk -= 8;   // space for R4-R11	
    return stk;
}
void OSInit(void) //ϵͳ��ʹ������ʹ�����������������ƿ�����Ĳ���
{ 
	  INT32U i;
	  OS_INT_OFF();             //�ر����ж�
    
	  for(i = 0; i < TIMER_SIZE; i++) {
	      OS_Sys.Timer[i]=0;
		}
		for(i = 0; i < FLAG_SIZE; i++) {
			  OS_Sys.FLAG[i]=OS_FALSE;
		}
		for(i = 0; i < MUTEX_SIZE; i++) {
			  OS_Sys.MUTEX[i]=OS_FALSE;
		}		
		for(i = 0; i < MBOX_SIZE; i++) {
		    OS_Sys.MBOX[i]=&Null;
		}
    for(i = 0; i < OS_MAX_TASKS; i++) { 
        OSTCBTbl[i].OSTCBStkPtr = (OS_STK*)0;
			  OSTCBTbl[i].OSTCBTaskAdd=0;
			  OSTCBTbl[i].OSTCBTaskNum=0;
			  OSTCBTbl[i].OSTCBTaskStr[0]='\0';
			  OSTCBTbl[i].OSTCBTaskTSCnt=0;
			  OSTCBTbl[i].OSTCBTaskCPUOccRate=0; 
        OSTCBTbl[i].OSTCBStat = TASK_CREATING;
    }
		OS_System.TaskTimeSliceCount=0;
		OS_Perip.OS_USART_COUNT=0;
		OS_System.OSTaskSwitchBack=0;
		OS_System.OSTaskNextRunFlag=OS_TRUE;
		OS_CreatTaskIdle();      //��������0  -��������
    OSCreatTaskManage();     //��������1  -�������������
		OS_System.OSTaskNext=0;  //Next���������ʹ��
		OSTCBRun = OSTCBCur;     //����TCBָ��Cur
    OSTCBCur = &OSTCBTbl[0]; //������0��ʼ����
    OSTCBNext = &OSTCBTbl[1];//��һ���񸳸�Next
		OS_System.OSRunning=OS_TRUE;       //OS������
		SysTickInit(OS_CLOCK_TIME);        //ϵͳ��ʱ����ʹ��
		OS_INT_ON();      //�����ж�
}
void OS_CreatTaskIdle(void)//������������(��������)
{
    OS_INT_OFF(); 
    OSTCBTbl[0].OSTCBStkPtr = OSTaskStkInit(OS_TaskIdle,(OS_STK*)&TASK_IDLE_STK[TaskIDLE_StkSize - 1]);
    OSTCBTbl[0].OSTCBStat = TASK_RUNNING;
	  OSTCBTbl[0].OSTCBTaskAdd=(INT32U)OS_TaskIdle;
    OS_INT_ON();
}
void OSCreatTaskManage(void)//�����������������(��������)
{
    OS_INT_OFF(); 
    OSTCBTbl[1].OSTCBStkPtr = OSTaskStkInit(OS_TaskManage, (OS_STK*)&TASK_MANAGE_STK[TaskManage_StkSize - 1]);
    OSTCBTbl[1].OSTCBStat = TASK_RUNNING;
	  OSTCBTbl[1].OSTCBTaskAdd=(INT32U)OS_TaskManage;
    OS_INT_ON();
}

void OSTaskCreate(char* taskstr,void (*task),OS_STK *p_tos,INT8U taskstate)//���񴴽�����(��������ڴ�����(��ջ)��)
{
	  INT16U i,j; 
	  OS_INT_OFF();
                                  

	  i=2;   //�������������������������񴴽��û�����
		while(OSTCBTbl[i].OSTCBStkPtr != (OS_STK*)0) {   //���ҿ�TCB
        i++;
    }
		for(j=0;j<TASK_NAME_SIZE;j++)
		{
			if(*taskstr=='\0') break;
		  OSTCBTbl[i].OSTCBTaskStr[j]=*taskstr++;
		} 
		OSTCBTbl[i].OSTCBStkPtr = OSTaskStkInit(task,p_tos);
    OSTCBTbl[i].OSTCBStat = taskstate;
		OSTCBTbl[i].OSTCBTaskAdd=(INT32U)task;
		OSTCBTbl[i].OSTCBTaskNum=i;
    OS_INT_ON();
}
void OS_TaskIdle(void) //�����������ݺ���  (���Է�ֹ0���������г���)
{
	while(1) 
	{
		
	}
}
void OS_TaskManage(void) //����������������ݺ���
{
	char* ptr;
	char* ptrbuff[OS_CMD_LEVEL+2];
	INT8U  i;
  while(1) 
	{
		OSSchedLock();
		if(OS_Perip.OS_USART_COUNT>1)
		{		
			if((OS_Perip.OS_USART_RX_BUFF[OS_Perip.OS_USART_COUNT-1]=='/')&&(OS_Perip.OS_USART_RX_BUFF[OS_Perip.OS_USART_COUNT-2]=='/'))
			{
				OS_Perip.OS_USART_COUNT=0;	
				ptr=strtok(OS_Perip.OS_USART_RX_BUFF,"/");
				ptrbuff[0]=ptr;
				for(i=1;i<OS_CMD_LEVEL+2;i++)
				{
				  if(ptr != NULL) ptrbuff[i]=strtok(NULL,"/");
				}
				if(strcmp(ptrbuff[0],"cmd") == 0) 
				{	
					/*********************�����һ��***************************/
					if(strcmp(ptrbuff[1],"open") == 0)        cmd.Level[1]=CMD_LEVEL_1_OPEN;
					if(strcmp(ptrbuff[1],"close") == 0)       cmd.Level[1]=CMD_LEVEL_1_CLOSE;
					if(strcmp(ptrbuff[1],"taskmanage") == 0)  cmd.Level[1]=CMD_LEVEL_1_TaskManage;

					/**********************************************************/
					/*********************����ڶ���***************************/
					if(strcmp(ptrbuff[2],"taskmanage") == 0)  cmd.Level[2]=CMD_LEVEL_2_TaskManage;
					if(strcmp(ptrbuff[2],"task") == 0)        cmd.Level[2]=CMD_LEVEL_2_Task;
			
					/***********************************************************/	
					/*********************���������***************************/
		
					
					/***********************************************************/	
					/***********************ָ���****************************/			
					 switch(cmd.Level[1])
					 {
						 case CMD_LEVEL_1_OPEN: 
							 switch(cmd.Level[2])
							 {
								 case CMD_LEVEL_2_TaskManage:   break;
								 case CMD_LEVEL_2_Task:		
									    for(i = 1; i < OS_MAX_TASKS; i++) {  
							          if(strcmp(ptrbuff[3],OSTCBTbl[i].OSTCBTaskStr) == 0) 
												  OSTaskStateSet((void*)OSTCBTbl[i].OSTCBTaskAdd,TASK_RUNNING);
												}					
								 break;
							 }
						 break;
						 case CMD_LEVEL_1_CLOSE: 
							 switch(cmd.Level[2])
							 {
								 case CMD_LEVEL_2_TaskManage:   break;
								 case CMD_LEVEL_2_Task:
									    
									    for(i = 1; i < OS_MAX_TASKS; i++) {  
							          if(strcmp(ptrbuff[3],OSTCBTbl[i].OSTCBTaskStr) == 0) 
												{
												  OSTaskStateSet((void*)OSTCBTbl[i].OSTCBTaskAdd,TASK_PAUSING);
													break;
												}
											}		
								 break;
							 }
						 break;
						 case CMD_LEVEL_1_TaskManage:  
									printf("*******************���������**********************  \r\r\n");
									printf("  TaskTimeSliceCount: %d \r\n",OS_System.TaskTimeSliceCount);
									printf("  CPUռ���ʣ�\r\r\n");
									for(i = 1; i < OS_MAX_TASKS; i++) 
									{
										if(i==1) printf("  Task_Manage : %0.1f%% \r\n",OSTCBTbl[i].OSTCBTaskCPUOccRate*1000/OS_System.TaskTimeSliceCount/10.0);
										if(OSTCBTbl[i].OSTCBTaskStr[0]!='\0')
										{					
											if(i>1) printf("  %s : %0.1f%% \r\n",OSTCBTbl[i].OSTCBTaskStr,OSTCBTbl[i].OSTCBTaskCPUOccRate*1000/OS_System.TaskTimeSliceCount/10.0);	
										}
									}	
									printf("***************************************************  \r\n");				 
							 switch(cmd.Level[2])
							 {
								 case CMD_LEVEL_2_TaskManage:   break;
								 case CMD_LEVEL_2_Task:   break;
							 }
						 break;
					 }
					
					/***********************************************************/				
				}
			}		
		}
	  OSSchedUnlock();	
	  OS_delayMs(1000);	
  }
}
void SysTickInit(INT16U Nus)  //��શ�ʱ����ʹ��(����OS�ļ�ʱ���壬OS��ʱ��Ƭ=n�����壬n�ɵ�)
{ 
	OS_INT_OFF();
	
	OS_System.TaskTimeSliceCount = 0;                            

	SysTick->LOAD  = (SystemCoreClock/8/1000000)* Nus;
	*Systick_priority = 0x00;
	SysTick->VAL   = 0;  
	SysTick->CTRL = 0x3; //ʹ���ⲿʱ�ӣ���8��Ƶ  72MHz/8=9MHz  ����9000��=1ms  ����900��=100us
	OS_INT_ON();
}
void SysTick_Handler(void)//�����л����ĺ���(��શ�ʱ�����жϴ��������������л�)
{
	  OS_INT_OFF();
		OSCoreSW(OS_MAX_TASKS,TIMER_SIZE);
    OS_INT_ON();  
}

INT8U OSTaskStateGet(void* Taskx) //��ȡ����״̬
{
	 INT8U i ;
	 INT8U stat;
	 for(i = 1; i < OS_MAX_TASKS; i++) {  
			 if( OSTCBTbl[i].OSTCBTaskAdd == (INT32U)Taskx ) 
			 {
				 stat= OSTCBTbl[i].OSTCBStat;
				 break;
			 }
		}
   return stat;		
}
void OSTaskStateSet(void* Taskx,INT8U TaskState) //����״̬���ú��� TaskState��TASK_RUNNING--����  TASK_PAUSING--��ͣ (�û��ɵ���)
{
	 INT8U i ;
	 OS_INT_OFF();  
	 if(OSTCBCur->OSTCBTaskAdd==(INT32U)Taskx) {
		 	if((OS_System.OSIntNesting == 0) && (OS_System.OSLockNesting == 0)) {	
				 OSTCBTbl[OSTCBCur->OSTCBTaskNum].OSTCBStat = TaskState; 
				 OSTCBCur = OSTCBNext;	 
				 OSCtxSw();
				}
		}
	 else{
		 for(i = 1; i < OS_MAX_TASKS; i++) {  
				 if( OSTCBTbl[i].OSTCBTaskAdd == (INT32U)Taskx ) 
				 {
						OSTCBTbl[i].OSTCBStat = TaskState;
					  break;
				 }
			}
	 }
	 OS_INT_ON(); 
}
void OSTaskSwitchBack(void* Taskx) //������ת ������
{
	 INT8U i;	
	 OS_INT_OFF(); 
   
	 if(OSTCBCur->OSTCBTaskAdd!=(INT32U)Taskx) 
	 {
		 for(i = 1; i < OS_MAX_TASKS; i++) {  
				 if( OSTCBTbl[i].OSTCBTaskAdd == (INT32U)Taskx ) 
				 {  
					 OS_System.OSTaskSwitchBack=OSTCBCur->OSTCBTaskNum;
					 OSTCBCur=&OSTCBTbl[i]; 
					 SysTick->VAL=0; 
					 OSCtxSw();					 
					 break;
				 }
		 }
	 }
	 OS_INT_ON(); 
}
void OSTaskSwitch(void* Taskx) //������ת  ��������
{
	 INT8U i;	
	 OS_INT_OFF(); 
   
	 if(OSTCBCur->OSTCBTaskAdd!=(INT32U)Taskx) 
	 {
		 for(i = 1; i < OS_MAX_TASKS; i++) {  
				 if( OSTCBTbl[i].OSTCBTaskAdd == (INT32U)Taskx ) 
				 {  
					 OS_System.OSTaskNext=i;
					 OSTCBNext = &OSTCBTbl[OS_System.OSTaskNext];
					 OSTCBCur=&OSTCBTbl[i];     
					 SysTick->VAL=0; 
					 OSCtxSw();					 
					 break;
				 }
		 }
	 }
	 OS_INT_ON(); 
}
void  OSSchedLock (void)   //�����л�����(�û��ɵ���) �����󲻻��л�����cpu��һֱ���е�ǰ����ֱ������
{
	if(OS_System.OSRunning == OS_TRUE)  
		{                                
			OS_INT_OFF();  
			if (OS_System.OSIntNesting == 0) 
				{                                          
					if (OS_System.OSLockNesting < 255u) 
						{         
							OS_System.OSLockNesting++;                     
					  }
			  }
		  OS_INT_ON();   
	  }
}
void  OSSchedUnlock (void)   //���������OSSchedLock��OSSchedUnlock����ɶԳ��֣�(�û��ɵ���)
{
	if (OS_System.OSRunning == OS_TRUE)
		{                            
			OS_INT_OFF();   
			if (OS_System.OSLockNesting > 0)  
				{                          
					OS_System.OSLockNesting--;   
					if (OS_System.OSLockNesting == 0)   
						{                       
							if (OS_System.OSIntNesting == 0)  
								{                 
									OS_INT_ON();  
									OSCtxSw();  
							  }            
								else 
								{
									OS_INT_ON(); 
							  }
					  }
					else
						{
							OS_INT_ON(); 
					  }
			  } 
			else
				{
					OS_INT_ON();
				}
		}
}
void OSIntEnter (void) //�����жϺ����������жϺ�����ͷ���� (�û��ɵ���)
{
	OS_INT_OFF(); 
	if (OS_System.OSIntNesting < 255)  
		{ 
	    OS_System.OSIntNesting++; 
	  }
	OS_INT_ON(); 
}
void OSIntExit (void) //�뿪�жϺ����������жϺ���ĩβ���ã�OSIntExit�����OSIntEnter�ɶԳ��֣� (�û��ɵ���)
{
	OS_INT_OFF();
	if (OS_System.OSIntNesting > 0) 
	  { 
	    OS_System.OSIntNesting--; 
	  }
	OS_INT_ON();
}
void OSTaskSwISR(void) //����жϺ���(��ʱ������)
{
   OSIntEnter();
   OSSchedLock();
	 	
	 OSSchedUnlock();
	 OSIntExit();	
} 

void OSFlagPost(uint16_t FNum)
{
  OS_Sys.FLAG[FNum]=OS_TRUE;
}
void OSFlagPend(uint16_t FNum)
{
  while(OS_Sys.FLAG[FNum]==OS_FALSE);
	OS_Sys.FLAG[FNum]=OS_FALSE;
}
void OSMutexPost(uint16_t MNum)
{
	OSSchedLock();
  if(OSTCBCur->OSTCBTaskNum==MutexOSTaskNum[MNum]) OS_Sys.MUTEX[MNum]=OS_FALSE;
	OSSchedUnlock();
}
void OSMutexPend(uint16_t MNum)
{
	while(OS_Sys.MUTEX[MNum]==OS_TRUE);
	OSSchedLock();
	OS_Sys.MUTEX[MNum]=OS_TRUE;
	MutexOSTaskNum[MNum]=OSTCBCur->OSTCBTaskNum;
	OSSchedUnlock();
}
void OSMboxPost(uint16_t MNum,void* fp)
{
  OS_Sys.MBOX[MNum]=fp;
}
void* OSMboxPend(uint16_t MNum)
{
	uint32_t* _mbox;
  while(*OS_Sys.MBOX[MNum]==Null);
	_mbox=(uint32_t*)OS_Sys.MBOX[MNum];
	OS_Sys.MBOX[MNum]=&Null;
	return _mbox;
}

INT32U GetOSSliceTime(void) //��ȡʱ����Ƭ������
{
    return OS_System.TaskTimeSliceCount;
}
INT32U GetOSTimerVal(void) //��ȡϵͳ��ʱ����ǰ����ֵ(Ĭ��Ϊ9MHz��ÿ�μ�����ʱ1/9 us)
{
   return SysTick->VAL;  //����ϵͳ��ʱ����ǰ����ֵ(�����û�us����ȷ��ʱ��������λΪϵͳ��ʱ����Ƶ���˴�ʹ����શ�ʱ������ƵΪ72MhzʱΪ9MHz��������Ƶʱ����8��Ƶ����)
}
void delay_us(INT32U nus) //΢����ʱ���� (�û��ɵ���)
{   	
	u32 Ticks,TCntOld,TCntNow,TCnt;
	u32 ReloadTimerValue=SysTick->LOAD; 
	SysTick->CTRL&=~(1<<1);//�ر�systick��ʱ���ж�
	      
	Ticks=nus*(SystemCoreClock/8/1000000);   
	TCnt=0;
	TCntOld=SysTick->VAL;
	while(1)
	{
			TCntNow=SysTick->VAL;  
			if(TCntNow!=TCntOld)
			{       
					if(TCntNow<TCntOld) TCnt += TCntOld-TCntNow;
					else TCnt += ReloadTimerValue-TCntNow+TCntOld;   
					TCntOld = TCntNow;
					if(TCnt>=Ticks) break;
			} 			
	} 
  SysTick->CTRL|=(1<<1); 	//��systick��ʱ���ж�
}
void delay_ms(INT32U nms)  //������ʱ���� (�û��ɵ���)
{     
	OSTCBCur->OSTCBTaskTSCnt=(nms*1000/OS_CLOCK_TIME)+1;
  while(OSTCBCur->OSTCBTaskTSCnt>1);
	OSTCBCur->OSTCBTaskTSCnt=0;
}
void OS_delayMs(volatile INT32U nms) //OS��ʱ����(�û��ɵ���)
{
	OS_INT_OFF();   
	OSTCBCur->OSTCBTaskTSCnt=(nms*1000/OS_CLOCK_TIME)+1;
	OSCtxSw(); 
	OS_INT_ON(); 
	while(OSTCBCur->OSTCBTaskTSCnt);
}


