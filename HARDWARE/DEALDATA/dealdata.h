#ifndef __DEALDATA_H
#define __DEALDATA_H 
#include "sys.h"

#define	DATAHEAD    0xDEED //����ͷ
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
	u8    Respond_Flag; //��Ӧ��־ 1����Ӧ 0������Ӧ
	
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
	SChassisAttitude,  				//������̬
	UploadData=0xF000					//�ϴ���������
	
}SetCMD;//��������


extern union TEMPDATA{
    s16  InTempData[15];
    u8 	 ChTempData[30];
}TempTxData,TempRxData;


void DealRXData(void);
void SendEncoderAndIMU20Ms(void);
#endif
