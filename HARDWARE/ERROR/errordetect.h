#ifndef __ERRORDETECT_H
#define __ERRORDETECT_H	 
#include "sys.h"



typedef struct
{
	//当前
	u16 NGYData[3];			 //陀螺仪数据
	u16 NAccelData[3];   //加速度计
	u16 NMagnetData[3];  //磁力计
	
	//上次
	u16 OGYData[3];			 //陀螺仪数据
	u16 OAccelData[3];   //加速度计
	u16 OMagnetData[3];  //磁力计
	
}IMUDATA;

extern IMUDATA  ImuData;
void Init_ErrorDetect(void);
void ErrorDetect(void);
#endif
