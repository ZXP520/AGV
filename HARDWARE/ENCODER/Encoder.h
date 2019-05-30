#ifndef __Encoder_H
#define	__Encoder_H
#include "stm32f10x.h"
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#define prd     EncoderPeriod
#define Vbreak  EncoderPeriod/2
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
typedef struct
{
	s16 V1;
	s16 V2;  //速度   实际值取决于读取的周期和编码器线数
	s16 V3;
	s16 V4;
	s16 V5;
	s16 cnt1;
	s16 cnt2;
	s16 cnt3;
	s16 cnt4;
	s16 cnt5;
	s16 rcnt1;
	s16 rcnt2;
	s16 rcnt3;
	s16 rcnt4;
	s16 rcnt5;
	s32 CNT1;
	s32 CNT2;
	s32 CNT3;
	s32 CNT4;
	s32 CNT5;
}EncoderType;
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
extern EncoderType GetEncoder;
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void Get_Encoder_T2(void);
void Get_Encoder_T3(void);
void Get_Encoder_T4(void);
void Get_Encoder_T5(void);
void Get_Encoder(void);
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#endif 

















