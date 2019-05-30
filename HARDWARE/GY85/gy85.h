#ifndef __GY85_H
#define __GY85_H
#include "sys.h"
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#define	SMPLRT_DIV		0x19	//�����ǲ����ʣ�����ֵ��0x07(125Hz)
#define	CONFIG			  0x1A	//��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
#define	ACCEL_CONFIG	0x1C	//���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)
#define	ACCEL_XOUT_H	0x32
#define	ACCEL_XOUT_L	0x33
#define	ACCEL_YOUT_H	0x34
#define	ACCEL_YOUT_L	0x35
#define	ACCEL_ZOUT_H	0x36
#define	ACCEL_ZOUT_L	0x37
#define	TEMP_OUT_H		0x1B
#define	TEMP_OUT_L		0x1C
#define	GYRO_XOUT_H		0x1D
#define	GYRO_XOUT_L		0x1E	
#define	GYRO_YOUT_H		0x1F
#define	GYRO_YOUT_L		0x20
#define	GYRO_ZOUT_H		0x21
#define	GYRO_ZOUT_L		0x22
#define	PWR_MGMT_1		0x3E	//��Դ��������ֵ��0x00(��������)
#define	WHO_AM_I			0x00	//IIC��ַ�Ĵ���(Ĭ����ֵ0x68��ֻ��)
#define	SlaveAddress	0xD0	//IICд��ʱ�ĵ�ַ�ֽ�����
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#define WHO	    0x00
#define	SMPL	0x15
#define DLPF	0x16
#define INT_C	0x17
#define INT_S	0x1A
#define	TMP_H	0x1B
#define	TMP_L	0x1C
#define	GX_H	0x00
#define	GX_L	0x01
#define	GY_H	0x02
#define	GY_L	0x03
#define GZ_H	0x04
#define GZ_L	0x05
#define PWR_M	0x3E
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#define	HMC5883L_Addr   0x1a	//�ų�������������ַ
#define	ADXL345_Addr    0xA6	//���ٶȴ�����������ַ
#define	ITG3050_Addr    0xD0	//�����Ǵ�����������ַ
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void 	InitGY85(void);
unsigned int GetData(uint8_t SlaveAddr,unsigned char REG_Address);
unsigned int GetQMC5883Data(uint8_t SlaveAddr,unsigned char REG_Address); //���16λ����
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#endif
