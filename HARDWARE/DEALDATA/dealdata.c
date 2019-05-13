#include "dealdata.h"
#include "sys.h" 
#include "delay.h"
#include "control.h"
#include "timer.h"
#include "usart.h"	
#include "mpu6050.h"

//数据测试函数
void TestSendData_To_Ros(void)
{
	int Cheksum=0;//校验和
	u8 i=0;
	
	TXData.InRxData[0]=DATAHEAD;  //头
	TXData.InRxData[1]=8;				//包长度	
	TXData.InRxData[2]=100;			//左轮速度
	TXData.InRxData[3]=-100;		//右轮速度
	TXData.InRxData[4]=1;				//停车信号
	TXData.InRxData[5]=0;				//导航标志
	TXData.InRxData[6]=3;				//陀螺仪轴数
	for(i=0;i<TXData.InRxData[1]-1;i++)
	{
		Cheksum+=TXData.InRxData[i];
	}
	TXData.InRxData[7]=Cheksum; //校验和
	
	for(i=0;i<TXData.InRxData[1]*4;i++)                         //循环发送数据
  {
      while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET); //循环发送,直到发送完毕   
        USART_SendData(USART2,TXData.ChRxData[i]); 
  } 
	
}

//20ms一次
void SendData_To_Ros(void)
{
	
	int Cheksum=0;//校验和
	u8 i=0;
	
	static u32 TempLeftEncoder_Cnt=0,TempRightEncoder_Cnt=0;
	
	//编码器脉冲个数计算
	if(LeftEncoder_Cnt>=TempLeftEncoder_Cnt)
	{
		TempLeftEncoder_Cnt=LeftEncoder_Cnt-TempLeftEncoder_Cnt;
	}
	else//溢出
	{
		TempLeftEncoder_Cnt=TempLeftEncoder_Cnt-LeftEncoder_Cnt;
	}
	
	if(RightEncoder_Cnt>=TempRightEncoder_Cnt)
	{
		TempRightEncoder_Cnt=RightEncoder_Cnt-TempRightEncoder_Cnt;
	}
	else//溢出
	{
		TempRightEncoder_Cnt=TempRightEncoder_Cnt-RightEncoder_Cnt;
	}
	
	TXData.InRxData[0]=DATAHEAD;  //头
	TXData.InRxData[1]=TXDATALENTH+AllWheel.imu_num;//包长度(8+imu轴数)	
	if(RightWheel.Direct)//正方向
	{
		TXData.InRxData[2]=TempRightEncoder_Cnt;						//右轮编码器
	}
	else
	{
		TXData.InRxData[2]=-TempRightEncoder_Cnt;						//右轮编码器
	}
	
	if(LeftWheel.Direct)//正方向
	{
		TXData.InRxData[3]=TempLeftEncoder_Cnt;					//左轮编码器
	}
	else
	{
		TXData.InRxData[3]=-TempLeftEncoder_Cnt;					//左轮编码器
	}
	TXData.InRxData[4]=Wheel_SPACING;	//电机驱动轮间距
	TXData.InRxData[5]=Wheel_D;				//轮子直径
	TXData.InRxData[6]=Wheel_RATIO;		//电机减速比
	
	//更新编码缓存器值
	TempLeftEncoder_Cnt	=LeftEncoder_Cnt ;
	TempRightEncoder_Cnt=RightEncoder_Cnt;
	//获得IMU数据
	switch(AllWheel.imu_num)
	{
		case 0: return;//为0则不发任何数据给ROS，初始默认为0
		case 3:
		{
			TXData.InRxData[7]=GetData(SlaveAddress,GYRO_XOUT_H);
			TXData.InRxData[8]=GetData(SlaveAddress,GYRO_YOUT_H);
			TXData.InRxData[9]=GetData(SlaveAddress,GYRO_ZOUT_H);		//陀螺仪数据
			break;
		}
		case 6:
		{
			TXData.InRxData[7]=GetData(SlaveAddress,GYRO_XOUT_H);
			TXData.InRxData[8]=GetData(SlaveAddress,GYRO_YOUT_H);
			TXData.InRxData[9]=GetData(SlaveAddress,GYRO_ZOUT_H);		//陀螺仪数据
			
			TXData.InRxData[10]=GetData(0xA6,ACCEL_XOUT_H);
			TXData.InRxData[11]=GetData(0xA6,ACCEL_YOUT_H);
			TXData.InRxData[12]=GetData(0xA6,ACCEL_ZOUT_H);					//加速度计数据
			break;
		}
		case 9:
		{
			TXData.InRxData[7]=GetData(SlaveAddress,GYRO_XOUT_H);
			TXData.InRxData[8]=GetData(SlaveAddress,GYRO_YOUT_H);
			TXData.InRxData[9]=GetData(SlaveAddress,GYRO_ZOUT_H);		//陀螺仪数据
			
			TXData.InRxData[10]=GetData(0xA6,ACCEL_XOUT_H);
			TXData.InRxData[11]=GetData(0xA6,ACCEL_YOUT_H);
			TXData.InRxData[12]=GetData(0xA6,ACCEL_ZOUT_H);					//加速度计数据
			
			TXData.InRxData[13]=0,TXData.InRxData[14]=0,TXData.InRxData[15]=0;//磁力计数据
			break;
		}
		default: break;
	}
	
	//计算校验值
	for(i=0;i<TXData.InRxData[1]-1;i++)
	{
		Cheksum+=TXData.InRxData[i];
	}
	TXData.InRxData[7]=Cheksum; //校验和
	
	for(i=0;i<TXData.InRxData[1]*4;i++)                         //循环发送数据
  {
      while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET); //循环发送,直到发送完毕   
        USART_SendData(USART2,TXData.ChRxData[i]); 
  } 
	
}

//处理接收到的数据，中断调用
void DealRXData(void)
{
	u8 i;
	int ChekSum=0;
	if(RXData.InRxData[0]==DATAHEAD && RXData.InRxData[1]==RXDATALENTH )//数据头和长度都对
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
