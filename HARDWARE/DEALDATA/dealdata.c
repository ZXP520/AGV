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

//处理接收到的数据，中断调用
DEALDATA_RX DealData_Rx;
//定义缓存共用体
union TEMPDATA TempTxData ,TempRxData;//发送缓存，接收缓存

/*******************************************************************************
* Function Name  : Respond_To_Ros
* Description    : 返回数据给ROS
* Input          : None 
* Output         : None
* Return         : None
****************************************************************************** */
static void Respond_To_Ros(void)
{
	s16 Cheksum=0;//校验和
	u8 i=0; 
	TXData.InRxData[0]=DATAHEAD;										//头
	TXData.ChRxData[2]=DealData_Rx.FrameLength;			//帧长度
	TXData.ChRxData[3]=DealData_Rx.CMD&0xFF;			//命令   小端模式先低后高
	TXData.ChRxData[4]=(DealData_Rx.CMD>>8)&0xFF;
	TXData.ChRxData[5]=DealData_Rx.Respond_Flag;
	TXData.ChRxData[6]=DealData_Rx.DataNum;					//数据个数	
	//小端模式，先发高位
	for(i=0;i<DealData_Rx.DataNum;i++)
	{
		TXData.ChRxData[7+i]=TempTxData.ChTempData[i];
	}
	
	//计算校验值
	for(i=0;i<DealData_Rx.FrameLength-2;i++)
	{
		Cheksum+=TXData.ChRxData[i];
	}
	TXData.ChRxData[DealData_Rx.FrameLength-2]=Cheksum&0xFF; //校验和
	TXData.ChRxData[DealData_Rx.FrameLength-1]=(Cheksum>>8)&0xFF;
	//DMA串口发送数据
	USART2_DMA_TX(TXData.ChRxData,DealData_Rx.FrameLength);
}


/*******************************************************************************
* Function Name  : DealRXData
* Description    : 每20MS上传编码器陀螺仪的数据
* Input          : None 
* Output         : None
* Return         : None 
****************************************************************************** */
void SendEncoderAndIMU20Ms(void)
{
	s16 Cheksum=0;//校验和
	static u8 i=0,Time_Cnt=0; 
	Time_Cnt++;
	
	//因为IIC读取数据时间长，所以分时取数据
	switch(Time_Cnt)
	{
		case 1:  //0ms
		{
			TXData.InRxData[0]=DATAHEAD;										//头
			TXData.ChRxData[2]=35;													//帧长度
			TXData.ChRxData[3]= UploadData    &0xFF;			  //命令   小端模式先低后高
			TXData.ChRxData[4]=(UploadData>>8)&0xFF;
			TXData.ChRxData[5]=0 ;
			TXData.ChRxData[6]=26;					//数据个数	18+8=26
			TempTxData.InTempData[0]=GetEncoder.V2;
			TempTxData.InTempData[1]=GetEncoder.V3;
			TempTxData.InTempData[2]=GetEncoder.V4;
			TempTxData.InTempData[3]=GetEncoder.V5;
			break;
		}
		case 2:  //5ms
		{
			TempTxData.InTempData[4]=GetData(ITG3050_Addr,GYRO_XOUT_H);
			TempTxData.InTempData[5]=GetData(ITG3050_Addr,GYRO_YOUT_H);
			TempTxData.InTempData[6]=GetData(ITG3050_Addr,GYRO_ZOUT_H);						//陀螺仪数据
			break;
		}
		case 3:  //10ms
		{
			TempTxData.InTempData[7]=GetData(ADXL345_Addr,ACCEL_XOUT_H);
			TempTxData.InTempData[8]=GetData(ADXL345_Addr,ACCEL_YOUT_H);
			TempTxData.InTempData[9]=GetData(ADXL345_Addr,ACCEL_ZOUT_H);					//加速度计数据
			break;
		}
		case 4:  //15ms
		{
			TempTxData.InTempData[10]=GetQMC5883Data(HMC5883L_Addr,GX_H);
			TempTxData.InTempData[11]=GetQMC5883Data(HMC5883L_Addr,GY_H);
			TempTxData.InTempData[12]=GetQMC5883Data(HMC5883L_Addr,GZ_H);					//磁力计数据
			
			break;
		}
		case 5:  //20ms
		{
			//小端模式，先发高位
			for(i=0;i<TXData.ChRxData[6];i++)
			{
				TXData.ChRxData[7+i]=TempTxData.ChTempData[i];
			}
			
			//计算校验值
			for(i=0;i<TXData.ChRxData[2]-2;i++)
			{
				Cheksum+=TXData.ChRxData[i];
			}
			TXData.ChRxData[TXData.ChRxData[2]-2]=Cheksum&0xFF; 				//校验和
			TXData.ChRxData[TXData.ChRxData[2]-1]=(Cheksum>>8)&0xFF;
			//DMA串口发送数据
			USART2_DMA_TX(TXData.ChRxData,TXData.ChRxData[2]);
			Time_Cnt=0;
			break;
		}
		default:break;
	}
	
	
	

	

	

	
}

/*******************************************************************************
* Function Name  : ExtractData
* Description    : 提取接收的数据
* Input          : None 
* Output         : None
* Return         : 数据校验结果 0错误数据 1正确数据
****************************************************************************** */
static u8 ExtractData(void)
{
	u8 i;
	u16 ChekSum=0;
	if(RXData.InRxData[0]==(s16)DATAHEAD )//数据头
	{
		//取得数据特征
		DealData_Rx.FrameLength=RXData.ChRxData[2];   						 //包长度
		DealData_Rx.CMD=(RXData.ChRxData[3] + (RXData.ChRxData[4]<<8));//命令     //先低后高
		DealData_Rx.Respond_Flag=RXData.ChRxData[5];               //响应标志
		DealData_Rx.DataNum=RXData.ChRxData[6];										 //数据个数
		switch(DealData_Rx.DataNum)
		{
			case 1:TempRxData.InTempData[0]=RXData.ChRxData[7];break;
			case 2:for(i=0;i<2;i++){TempRxData.ChTempData[i]=RXData.ChRxData[7+i];}break;
			case 4:for(i=0;i<4;i++){TempRxData.ChTempData[i]=RXData.ChRxData[7+i];}break;
			case 6:for(i=0;i<6;i++){TempRxData.ChTempData[i]=RXData.ChRxData[7+i];}break;
			case 8:for(i=0;i<8;i++){TempRxData.ChTempData[i]=RXData.ChRxData[7+i];}break; //速度设置最多为8
			default:break;
		}
		DealData_Rx.CheckSum=(RXData.ChRxData[DealData_Rx.FrameLength-1]<<8)+RXData.ChRxData[DealData_Rx.FrameLength-2];
		
		for(i=0;i<DealData_Rx.FrameLength-2;i++)
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
	return 0;
}

/*******************************************************************************
* Function Name  : DealRXData
* Description    : 处理接收的数据
* Input          : None 
* Output         : None
* Return         : None 
****************************************************************************** */
void DealRXData(void)
{
	if(ExtractData()){;}
	else{return;}
	switch(DealData_Rx.CMD)//命令解析
	{
		//查询指令
		case WhellDiameter:  //轮子直径
		{
			TempTxData.InTempData[0]=Wheel_D;
			break;    
		}
		case WheelBase:      //轮子轴距
		{
			TempTxData.InTempData[0]=Wheel_SPACING;
			break;
		}
		case WhellRollSpeed: //轮子转速
		{
			
			break;
		}
		case WhellSpeed:     //轮子速度
		{		
			
			break;
		}
		case ReductionRatio: //轮子减速比
		{
			TempTxData.InTempData[0]=Wheel_RATIO;
			break;
		}
		case WhellAcceleration://轮子加速度
		{	
			break;
		}
			
		case EncoderLine:      //编码器线数
		{	
			TempTxData.InTempData[0]=ENCODER_LINE;
			break;
		}
		case EncoderValue:     //编码器值
		{
		#if    VERSION==0
			TempTxData.InTempData[0]=GetEncoder.V5;
			TempTxData.InTempData[1]=GetEncoder.V3;
		#elif  VERSION==1
			TempTxData.InTempData[0]=GetEncoder.V3;
			TempTxData.InTempData[1]=GetEncoder.V4;
			TempTxData.InTempData[2]=GetEncoder.V5;
		#endif
			break;
		}
		case IMUData:     		 //陀螺仪数据
		{
			switch(DealData_Rx.DataNum)
			{
				case 6:  //3轴
				{
					TempTxData.InTempData[0]=GetData(ITG3050_Addr,GYRO_XOUT_H);
					TempTxData.InTempData[1]=GetData(ITG3050_Addr,GYRO_YOUT_H);
					TempTxData.InTempData[2]=GetData(ITG3050_Addr,GYRO_ZOUT_H);						//陀螺仪数据
					break;
				}
				case 12: //6轴
				{
					TempTxData.InTempData[0]=GetData(ITG3050_Addr,GYRO_XOUT_H);
					TempTxData.InTempData[1]=GetData(ITG3050_Addr,GYRO_YOUT_H);
					TempTxData.InTempData[2]=GetData(ITG3050_Addr,GYRO_ZOUT_H);						//陀螺仪数据
			
					TempTxData.InTempData[3]=GetData(ADXL345_Addr,ACCEL_XOUT_H);
					TempTxData.InTempData[4]=GetData(ADXL345_Addr,ACCEL_YOUT_H);
					TempTxData.InTempData[5]=GetData(ADXL345_Addr,ACCEL_ZOUT_H);					//加速度计数据
					break;
				}
				case 18:  //9轴
				{
					TempTxData.InTempData[0]=GetData(ITG3050_Addr,GYRO_XOUT_H);
					TempTxData.InTempData[1]=GetData(ITG3050_Addr,GYRO_YOUT_H);
					TempTxData.InTempData[2]=GetData(ITG3050_Addr,GYRO_ZOUT_H);						//陀螺仪数据
			
					TempTxData.InTempData[3]=GetData(ADXL345_Addr,ACCEL_XOUT_H);
					TempTxData.InTempData[4]=GetData(ADXL345_Addr,ACCEL_YOUT_H);
					TempTxData.InTempData[5]=GetData(ADXL345_Addr,ACCEL_ZOUT_H);					//加速度计数据
			
					TempTxData.InTempData[6]=GetQMC5883Data(HMC5883L_Addr,GX_H);
					TempTxData.InTempData[7]=GetQMC5883Data(HMC5883L_Addr,GY_H);
					TempTxData.InTempData[8]=GetQMC5883Data(HMC5883L_Addr,GZ_H);					//磁力计数据
					break;
				}
				default:break;
			}
			break;
		}
		case UltrasonicData: 	 //超声波数据
		{
			break;
		}
		case EmergencyStop:    //急停状态
		{
			TempTxData.ChTempData[0]=AllWheel.stop_flag;
			break;
		}
		case VersionNumber:    //版本号
		{
			break;
		}
		case RemainingBattery: //剩余电量
		{
			break;  
		}
		
		//设置命令
		case SWhellRollSpeed:   //轮子转速
		{
			break;  
		}
		case SWhellSpeed: 			//轮子速度
		{
			AllWheel.stop_flag=0;
			LeftWheelSpeedSet	(TempRxData.InTempData[0]);
		  RightWheelSpeedSet(TempRxData.InTempData[1]);
			break;
		}
		case STurningRadius:    //拐弯半径
		{
			break;
		}
		case SWhellAcceleration://轮子加速度
		{
			break;
		}
		case SChassisAttitude:  //底盘姿态
		{
			AllWheel.stop_flag=0;
			OmniWheelscontrol(TempRxData.InTempData[0],TempRxData.InTempData[1],TempRxData.InTempData[2],0);
			break;
		}
		//正确的命令才响应
		default:return;
	}
	
	//响应标志为1才响应
	if(DealData_Rx.Respond_Flag)
	{
		Respond_To_Ros();
	}
}
