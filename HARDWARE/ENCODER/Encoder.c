#include "Encoder.h"
#include "timer.h"
#include <math.h>


/*******************************±àÂëÆ÷¶ÁÈ¡**************************
*/

EncoderType GetEncoder;

void Get_Encoder_T2(void)
{
  s32 CNT2_temp,CNT2_last;
  
  GetEncoder.cnt2 = TIM2 -> CNT;
  CNT2_last = GetEncoder.CNT2;
  CNT2_temp = GetEncoder.rcnt2 * prd + GetEncoder.cnt2;  
  GetEncoder.V2 = CNT2_temp - CNT2_last;		
  
  while ((s32)(GetEncoder.V2)>Vbreak)				 
  {							      
   GetEncoder.rcnt2--;					      
   CNT2_temp = GetEncoder.rcnt2 * prd + GetEncoder.cnt2;
   GetEncoder.V2 = CNT2_temp - CNT2_last;		 
  }							     
  while ((s32)(GetEncoder.V2)<-Vbreak)			   
  {							      
   GetEncoder.rcnt2++;					      
   CNT2_temp = GetEncoder.rcnt2 * prd + GetEncoder.cnt2;
   GetEncoder.V2 = CNT2_temp - CNT2_last;		 
  }
  GetEncoder.CNT2 = CNT2_temp;				 
}


///////////////////
void Get_Encoder_T3(void)
{
  s32 CNT3_temp,CNT3_last;
 
  GetEncoder.cnt3 = TIM3 -> CNT;
  CNT3_last = GetEncoder.CNT3;
  CNT3_temp = GetEncoder.rcnt3 * prd + GetEncoder.cnt3; 
  GetEncoder.V3 = CNT3_temp - CNT3_last;		
  
  while ((s32)(GetEncoder.V3)>Vbreak)				 
  {							      
   GetEncoder.rcnt3--;					      
   CNT3_temp = GetEncoder.rcnt3 * prd + GetEncoder.cnt3;
   GetEncoder.V3 = CNT3_temp - CNT3_last;		 
  }							     
  while ((s32)(GetEncoder.V3)<-Vbreak)			   
  {							      
   GetEncoder.rcnt3++;					      
   CNT3_temp = GetEncoder.rcnt3 * prd + GetEncoder.cnt3;
   GetEncoder.V3 = CNT3_temp - CNT3_last;		 
  }
  GetEncoder.CNT3 = CNT3_temp;		
}


///////////////////
void Get_Encoder_T4(void)
{
  s32 CNT4_temp,CNT4_last;
  
  GetEncoder.cnt4 = TIM4 -> CNT;
  CNT4_last = GetEncoder.CNT4;
  CNT4_temp = GetEncoder.rcnt4 * prd + GetEncoder.cnt4;  
  GetEncoder.V4 = CNT4_temp - CNT4_last;		 
  
  while ((s32)(GetEncoder.V4)>Vbreak)				 
  {							      
   GetEncoder.rcnt4--;					     
   CNT4_temp = GetEncoder.rcnt4 * prd + GetEncoder.cnt4;
   GetEncoder.V4 = CNT4_temp - CNT4_last;		 
  }							      
  while ((s32)(GetEncoder.V4)<-Vbreak)			   
  {							    
   GetEncoder.rcnt4++;					      
   CNT4_temp = GetEncoder.rcnt4 * prd + GetEncoder.cnt4;
   GetEncoder.V4 = CNT4_temp - CNT4_last;		  
  }
  GetEncoder.CNT4 = CNT4_temp;		
}


///////////////////
void Get_Encoder_T5(void)
{
  s32 CNT5_temp,CNT5_last;
  
  GetEncoder.cnt5 = TIM5 -> CNT;
  CNT5_last = GetEncoder.CNT5;
  CNT5_temp = GetEncoder.rcnt5 * prd + GetEncoder.cnt5;  
  GetEncoder.V5 = CNT5_temp - CNT5_last;		  
  
  while ((s32)(GetEncoder.V5)>Vbreak)				  
  {							      
   GetEncoder.rcnt5--;					      
   CNT5_temp = GetEncoder.rcnt5 * prd + GetEncoder.cnt5;
   GetEncoder.V5 = CNT5_temp - CNT5_last;		 
  }
  while ((s32)(GetEncoder.V5)<-Vbreak)			    
  {							     
   GetEncoder.rcnt5++;					      
   CNT5_temp = GetEncoder.rcnt5 * prd + GetEncoder.cnt5;
   GetEncoder.V5 = CNT5_temp - CNT5_last;		 
  }
  GetEncoder.CNT5 = CNT5_temp;
}


void Get_Encoder(void)
{
//	Get_Encoder_T2();
  Get_Encoder_T3();
//  Get_Encoder_T4();
  Get_Encoder_T5();
}









