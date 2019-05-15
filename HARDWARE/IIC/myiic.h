#ifndef __MYIIC_H
#define __MYIIC_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//IIC���� ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/9
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////

//IO��������
/*ģ��IIC�˿�������붨��*/
#define SCL_H         GPIOB->BSRR = GPIO_Pin_4
#define SCL_L         GPIOB->BRR  = GPIO_Pin_4 
   
#define SDA_H         GPIOB->BSRR = GPIO_Pin_5
#define SDA_L         GPIOB->BRR  = GPIO_Pin_5

#define SCL_read      GPIOB->IDR  & GPIO_Pin_4
#define SDA_read      GPIOB->IDR  & GPIO_Pin_5



void I2C_GPIO_Config(void);
u8 IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	  
#endif
















