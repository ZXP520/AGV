#ifndef __DEALDATA_H
#define __DEALDATA_H 
#include "sys.h"

#define	DATAHEAD    0xA5A5 //数据头
#define RXDATALENTH 0x0A //接收数据长度
#define TXDATALENTH 0x0008 //发送数据长度

//数据接收结构体
typedef struct
{
	_Bool Success_Flag;//接收成功标志
	u8    DataCMD;			//数据命令
	s16   Data[4];			//接收数据
	
	
}DEALDATA_RX;
extern DEALDATA_RX DealData_Rx;

void SendData_To_Ros(void);
void TestSendData_To_Ros(void);
void DealRXData(void);
#endif
