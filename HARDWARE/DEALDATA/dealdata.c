#include "dealdata.h"
#include "sys.h" 
#include "delay.h"
#include "control.h"
#include "timer.h"
#include "usart.h"	
#include "gy85.h"
#include "Encoder.h"
#include "bsp_usart.h"
#include "include.h"

//数据测试函数
void TestSendData_To_Ros(void)
{
	int Cheksum=0;//校验和
	u8 i=0;
	
	TXData.InRxData[0]=DATAHEAD;  //头
	TXData.InRxData[1]=10;				//包长度	
	TXData.InRxData[2]=100;			  //左轮速度
	TXData.InRxData[3]=100;		    //右轮速度
	TXData.InRxData[4]=0;			    //前左轮
	TXData.InRxData[5]=0;		      //前右轮
	TXData.InRxData[6]=0;				  //停车信号
	TXData.InRxData[7]=0;				  //导航标志
	TXData.InRxData[8]=3;				  //陀螺仪轴数
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
			TXData.InRxData[9]=GetData(ITG3050_Addr,GYRO_ZOUT_H);						//陀螺仪数据
			break;
		}
		case 6:
		{
			TXData.InRxData[7]=GetData(ITG3050_Addr,GYRO_XOUT_H);
			TXData.InRxData[8]=GetData(ITG3050_Addr,GYRO_YOUT_H);
			TXData.InRxData[9]=GetData(ITG3050_Addr,GYRO_ZOUT_H);						//陀螺仪数据
			
			TXData.InRxData[10]=GetData(ADXL345_Addr,ACCEL_XOUT_H);
			TXData.InRxData[11]=GetData(ADXL345_Addr,ACCEL_YOUT_H);
			TXData.InRxData[12]=GetData(ADXL345_Addr,ACCEL_ZOUT_H);					//加速度计数据
			break;
		}
		case 9:
		{
			TXData.InRxData[7]=GetData(ITG3050_Addr,GYRO_XOUT_H);
			TXData.InRxData[8]=GetData(ITG3050_Addr,GYRO_YOUT_H);
			TXData.InRxData[9]=GetData(ITG3050_Addr,GYRO_ZOUT_H);						//陀螺仪数据
			
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

//处理小速度问题小于8就为0
s16 DealSmallData(s16 data)
{
	static s16 temp=0;
	temp = data;
	if(temp<8&&temp>-8)
	{
		temp=0;
	}
	return temp;
}


//提取数据
u8 ExtractData(void)
{
	u8 i;
	u16 ChekSum=0;
	if(RXData.InRxData[0]==(s16)DATAHEAD )//数据头
	{
		//取得数据特征
		DealData_Rx.FrameLength=RXData.ChRxData[2];
		DealData_Rx.CMD=(RXData.ChRxData[3]<<8)+RXData.ChRxData[4];
		DealData_Rx.DataNum=RXData.ChRxData[5];
		DealData_Rx.CheckSum=(RXData.ChRxData[DealData_Rx.FrameLength-2]<<8)+RXData.ChRxData[DealData_Rx.FrameLength-1];
		
		for(i=0;i<DealData_Rx.FrameLength-1;i++)
		{
			ChekSum+=RXData.ChRxData[i];
		}
		if(ChekSum == DealData_Rx.CheckSum)//数据校验正确
		{
			DealData_Rx.Success_Flag=1;
			return 1;
		}
		else
		{
			DealData_Rx.Success_Flag=0;
			return 0;
		}
	}
}

//处理接收到的数据，中断调用
DEALDATA_RX DealData_Rx;
void DealRXData(void)
{
	
	if(ExtractData()){;}
	else{return;}
	switch(DealData_Rx.CMD)//命令解析
	{
		case 0x00:break;
		case 0x01:break;
		case 0x02:    //差速两轮
		{
			LeftWheelSpeedSet	(DealSmallData(RXData.InRxData[3]));//设置左轮速度
			RightWheelSpeedSet(DealSmallData(RXData.InRxData[4]));//设置右轮速度
			break;
		}
		case 0x03:		//全向三轮
		{
			OmniWheelscontrol(DealSmallData(RXData.InRxData[3]),DealSmallData(RXData.InRxData[4]),DealSmallData(RXData.InRxData[5]),DealSmallData(RXData.InRxData[6]));
			break;
		}
		default:break;
	}

	AllWheel.stop_flag			=RXData.ChRxData[15];//停车标志
	AllWheel.navigation_flag=RXData.ChRxData[16];//导航标志
	AllWheel.imu_num				=RXData.InRxData[8]; //陀螺仪数据轴数
	DealData_Rx.Success_Flag=1;  										 //数据接收成功
	
}
