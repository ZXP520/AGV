#ifndef __DEALDATA_H
#define __DEALDATA_H 
#include "sys.h"

#define	DATAHEAD  0x0000A5A5 //����ͷ
#define RXDATALENTH 0x00000008 //�������ݳ���
#define TXDATALENTH 0x00000008 //�������ݳ���
void SendData_To_Ros(void);
void TestSendData_To_Ros(void);
void DealRXData(void);
#endif
