#ifndef __DEALDATA_H
#define __DEALDATA_H 
#include "sys.h"

#define	DATAHEAD    0xA5A5 //����ͷ
#define RXDATALENTH 0x0A //�������ݳ���
#define TXDATALENTH 0x0008 //�������ݳ���
void SendData_To_Ros(void);
void TestSendData_To_Ros(void);
void DealRXData(void);
#endif
