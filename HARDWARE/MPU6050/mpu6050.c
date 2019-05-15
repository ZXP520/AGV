#include "mpu6050.h"
#include "myiic.h"

void InitMPU6050(void) //��ʼ��Mpu6050
{
	 IIC_Init();
	 Single_Write(0x1A,0x0B,0x01); 
   Single_Write(0x1A,0x20,0x40);
   Single_Write(0x1A,0x21,0x01);
   Single_Write(0x1A,0x09,0x0d); //
	
	 Single_Write(0xA6,0x31,0x0B);   //������Χ,����16g��13λģʽ
   Single_Write(0xA6,0x2D,0x08);   //ѡ���Դģʽ   �ο�pdf24ҳ
   Single_Write(0xA6,0x2E,0x80);   //ʹ�� DATA_READY �ж�
	
	 Single_Write(SlaveAddress,PWR_M, 0x80);   //
   Single_Write(SlaveAddress,SMPL, 0x07);    //
   Single_Write(SlaveAddress,DLPF, 0x1E);    //��2000��
   Single_Write(SlaveAddress,INT_C, 0x00 );  //
   Single_Write(SlaveAddress,PWR_M, 0x00);   //
	 
}

unsigned int GetData(uint8_t SlaveAddr,unsigned char REG_Address) //���16λ����
{
	char H,L;
	H=GetData(SlaveAddr,REG_Address);
	L=GetData(SlaveAddr,REG_Address+1);
	return (H<<8)+L;   //�ϳ�����
}

unsigned int GetQMC5883Data(uint8_t SlaveAddr,unsigned char REG_Address) //���16λ����
{
	char H,L;
	L=GetData(SlaveAddr,REG_Address);
	H=GetData(SlaveAddr,REG_Address+1);
	return (H<<8)+L;   //�ϳ�����
}




