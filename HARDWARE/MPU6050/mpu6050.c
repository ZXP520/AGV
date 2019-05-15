#include "mpu6050.h"
#include "myiic.h"

void InitMPU6050(void) //��ʼ��Mpu6050
{
	 I2C_GPIO_Config();
	
	 IIC_Write_One_Byte(0x1A,0x0B,0x01); 
   IIC_Write_One_Byte(0x1A,0x20,0x40);
   IIC_Write_One_Byte(0x1A,0x21,0x01);
   IIC_Write_One_Byte(0x1A,0x09,0x0d); //
	
	 IIC_Write_One_Byte(0xA6,0x31,0x0B);   //������Χ,����16g��13λģʽ
   IIC_Write_One_Byte(0xA6,0x2D,0x08);   //ѡ���Դģʽ   �ο�pdf24ҳ
   IIC_Write_One_Byte(0xA6,0x2E,0x80);   //ʹ�� DATA_READY �ж�
	
	 IIC_Write_One_Byte(SlaveAddress,PWR_M, 0x80);   //
   IIC_Write_One_Byte(SlaveAddress,SMPL, 0x07);    //
   IIC_Write_One_Byte(SlaveAddress,DLPF, 0x1E);    //��2000��
   IIC_Write_One_Byte(SlaveAddress,INT_C, 0x00 );  //
   IIC_Write_One_Byte(SlaveAddress,PWR_M, 0x00);   //
	 
}

unsigned int GetData(uint8_t SlaveAddr,unsigned char REG_Address) //���16λ����
{
	char H,L;
	H=IIC_Read_One_Byte(SlaveAddr,REG_Address);
	L=IIC_Read_One_Byte(SlaveAddr,REG_Address+1);
	return (H<<8)+L;   //�ϳ�����
}

unsigned int GetQMC5883Data(uint8_t SlaveAddr,unsigned char REG_Address) //���16λ����
{
	char H,L;
	L=IIC_Read_One_Byte(SlaveAddr,REG_Address);
	H=IIC_Read_One_Byte(SlaveAddr,REG_Address+1);
	return (H<<8)+L;   //�ϳ�����
}




