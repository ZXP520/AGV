#include "dealdata.h"
#include "sys.h" 
#include "delay.h"
#include "control.h"
#include "timer.h"
#include "usart.h"	
#include "gy85.h"
#include "Encoder.h"
#include "bsp_usart.h"

//数据测试函数
void TestSendData_To_Ros(void)
{
	int Cheksum=0;//校验和
	u8 i=0;
	
	TXData.InRxData[0]=DATAHEAD;  //头
	TXData.InRxData[1]=8;				//包长度	
	TXData.InRxData[2]=100;			//左轮速度
	TXData.InRxData[3]=100;		//右轮速度
	TXData.InRxData[4]=0;				//停车信号
	TXData.InRxData[5]=0;				//导航标志
	TXData.InRxData[6]=3;				//陀螺仪轴数
	for(i=0;i<TXData.InRxData[1]-1;i++)
	{
		Cheksum+=TXData.InRxData[i];
	}
	TXData.InRxData[7]=Cheksum; //校验和
	
	//DMA串口发送数据
	USART2_DMA_TX(TXData.ChRxData,TXData.InRxData[1]*2);
	
}

//20ms一次
void SendData_To_Ros(void)
{
	
	int Cheksum=0;//校验和
	u8 i=0;
		
	TXData.InRxData[0]=DATAHEAD;  //头
	TXData.InRxData[1]=TXDATALENTH+AllWheel.imu_num;//包长度(8+imu轴数)	
	TXData.InRxData[2]=GetEncoder.V3;								//右轮编码器
	TXData.InRxData[3]=GetEncoder.V5;								//左轮编码器
	TXData.InRxData[4]=Wheel_SPACING;								//电机驱动轮间距
	TXData.InRxData[5]=Wheel_D;											//轮子直径
	TXData.InRxData[6]=Wheel_RATIO;									//电机减速比
	//获得IMU数据
	
	switch(AllWheel.imu_num)
	{
		case 0: return;//为0则不发任何数据给ROS，初始默认为0
		case 3:
		{
			TXData.InRxData[7]=GetData(ITG3050_Addr,GYRO_XOUT_H);
			TXData.InRxData[8]=GetData(ITG3050_Addr,GYRO_YOUT_H);
			TXData.InRxData[9]=GetData(ITG3050_Addr,GYRO_ZOUT_H);		//陀螺仪数据
			break;
		}
		case 6:
		{
			TXData.InRxData[7]=GetData(ITG3050_Addr,GYRO_XOUT_H);
			TXData.InRxData[8]=GetData(ITG3050_Addr,GYRO_YOUT_H);
			TXData.InRxData[9]=GetData(ITG3050_Addr,GYRO_ZOUT_H);		//陀螺仪数据
			
			TXData.InRxData[10]=GetData(ADXL345_Addr,ACCEL_XOUT_H);
			TXData.InRxData[11]=GetData(ADXL345_Addr,ACCEL_YOUT_H);
			TXData.InRxData[12]=GetData(ADXL345_Addr,ACCEL_ZOUT_H);					//加速度计数据
			break;
		}
		case 9:
		{
			TXData.InRxData[7]=GetData(ITG3050_Addr,GYRO_XOUT_H);
			TXData.InRxData[8]=GetData(ITG3050_Addr,GYRO_YOUT_H);
			TXData.InRxData[9]=GetData(ITG3050_Addr,GYRO_ZOUT_H);		//陀螺仪数据
			
			TXData.InRxData[10]=GetData(ADXL345_Addr,ACCEL_XOUT_H);
			TXData.InRxData[11]=GetData(ADXL345_Addr,ACCEL_YOUT_H);
			TXData.InRxData[12]=GetData(ADXL345_Addr,ACCEL_ZOUT_H);					//加速度计数据
			
			TXData.InRxData[13]=GetQMC5883Data(HMC5883L_Addr,GX_H);
			TXData.InRxData[14]=GetQMC5883Data(HMC5883L_Addr,GY_H);
			TXData.InRxData[15]=GetQMC5883Data(HMC5883L_Addr,GZ_H);					//磁力计数据
			break;
		}
		default: break;
	}
	
	//计算校验值
	for(i=0;i<TXData.InRxData[1]-1;i++)
	{
		Cheksum+=TXData.InRxData[i];
	}
	TXData.InRxData[TXData.InRxData[1]-1]=Cheksum; //校验和
	//DMA串口发送数据
	USART2_DMA_TX(TXData.ChRxData,TXData.InRxData[1]*2);
	
}

//处理接收到的数据，中断调用
void DealRXData(void)
{
	u8 i;
	int ChekSum=0;
	if(RXData.InRxData[0]==(s16)DATAHEAD && RXData.InRxData[1]==(s16)RXDATALENTH )//数据头和长度都对
	{
		for(i=0;i<RXData.InRxData[1]-1;i++)
		{
			ChekSum+=RXData.InRxData[i];
		}
		if(ChekSum == RXData.InRxData[7])//数据校验正确
		{
			RightWheelSpeedSet(RXData.InRxData[2]);//设置右轮速度
			LeftWheelSpeedSet	(RXData.InRxData[3]);//设置左轮速度
			AllWheel.stop_flag			=RXData.InRxData[4];//停车标志
			AllWheel.navigation_flag=RXData.InRxData[5];//导航标志
			AllWheel.imu_num				=RXData.InRxData[6];//陀螺仪数据轴数
		}
	}
}
