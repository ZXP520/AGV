#ifndef __DEALDATA_H
#define __DEALDATA_H 
#include "sys.h"

#define	DATAHEAD    0xDEDE //����ͷ
#define RXDATALENTH 0x0A //�������ݳ���
#define TXDATALENTH 0x0008 //�������ݳ���

//���ݽ��սṹ��
typedef struct
{
	_Bool Success_Flag;	//���ճɹ���־
	u8    FrameLength;		//֡����
	u16   CMD;					//��������
	u8    DataNum;				//�������
	s16   CheckSum;			//У����
	
}DEALDATA_RX;
extern DEALDATA_RX DealData_Rx;

typedef enum 
{
	WhellDiameter=0,	//����ֱ��
	WheelBase,				//�������
	WhellRollSpeed,   //����ת��
	WhellSpeed,				//�����ٶ�
	ReductionRatio,   //���Ӽ��ٱ�
	WhellAcceleration,//���Ӽ��ٶ�
	EncoderLine, 			//����������
	EncoderValue,     //������ֵ
	IMUData,					//����������
	UltrasonicData,   //����������
	EmergencyStop,    //��ͣ״̬
	VersionNumber,    //�汾��
	RemainingBattery  //ʣ�����
	
}InquireCMD;//��ѯ����



typedef enum 
{
	SWhellRollSpeed=0x8000,   //����ת��
	SWhellSpeed,							//�����ٶ�
	STurningRadius,     			//���ӹ���뾶
	SWhellAcceleration,       //���Ӽ��ٶ�
	SChassisAttitude  				//������̬
	
}SetCMD;//��������



void SendData_To_Ros(void);
void TestSendData_To_Ros(void);
void DealRXData(void);
#endif
