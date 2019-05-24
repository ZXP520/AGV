#ifndef __DEALDATA_H
#define __DEALDATA_H 
#include "sys.h"

#define	DATAHEAD    0xA5A5 //����ͷ
#define RXDATALENTH 0x0A //�������ݳ���
#define TXDATALENTH 0x0008 //�������ݳ���

//���ݽ��սṹ��
typedef struct
{
	_Bool Success_Flag;//���ճɹ���־
	u8    DataCMD;			//��������
	s16   Data[4];			//��������
	
	
}DEALDATA_RX;
extern DEALDATA_RX DealData_Rx;

void SendData_To_Ros(void);
void TestSendData_To_Ros(void);
void DealRXData(void);
#endif
