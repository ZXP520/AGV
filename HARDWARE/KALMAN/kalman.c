#include "kalman.h"
#include "gy85.h"
#include "math.h"

float Accel_x;	     //X����ٶ�ֵ�ݴ�
float Accel_y;	     //Y����ٶ�ֵ�ݴ�
float Accel_z;	     //Z����ٶ�ֵ�ݴ�

float Gyro_x;		 //X�������������ݴ�
float Gyro_y;        //Y�������������ݴ�
float Gyro_z;		 //Z�������������ݴ�

float Bmp_x;
float Bmp_y;
float Bmp_z;

//float Angle_gy;    //�ɽ��ٶȼ������б�Ƕ�
float Angle_x_temp;  //�ɼ��ٶȼ����x��б�Ƕ�
float Angle_y_temp;  //�ɼ��ٶȼ����y��б�Ƕ�

float Angle_X_Final; //X������б�Ƕ�
float Angle_Y_Final; //Y������б�Ƕ�

//�Ƕȼ���
void Angle_Calcu(void)	 
{
	static int yaw=0;
	//��ΧΪ2gʱ�������ϵ��16384 LSB/g
	//deg = rad*180/3.14
	float x,y,z;
	
	Accel_x = GetData(0xA6,ACCEL_XOUT_H); //x����ٶ�ֵ�ݴ�
	Accel_y = GetData(0xA6,ACCEL_YOUT_H); //y����ٶ�ֵ�ݴ�
	Accel_z = GetData(0xA6,ACCEL_ZOUT_H); //z����ٶ�ֵ�ݴ�
	Gyro_x  = GetData(SlaveAddress,GYRO_XOUT_H);  //x��������ֵ�ݴ�
	Gyro_y  = GetData(SlaveAddress,GYRO_YOUT_H);  //y��������ֵ�ݴ�
	Gyro_z  = GetData(SlaveAddress,GYRO_ZOUT_H);  //z��������ֵ�ݴ�
	
	
	//Bmp_x   = GetData(0x3C,0x03);
	//Bmp_y		= GetData(0x3C,0x05);
	//Bmp_z		= GetData(0x3C,0x07);
	
	yaw += Gyro_z*0.001/32.768f;
	
	//u2_printf("%d %d \n",(int)Gyro_z,yaw);
	
	//����x����ٶ�
	if(Accel_x<32764) x=Accel_x/16384;
	else              x=1-(Accel_x-49152)/16384;
	
	//����y����ٶ�
	if(Accel_y<32764) y=Accel_y/16384;
	else              y=1-(Accel_y-49152)/16384;
	
	//����z����ٶ�
	if(Accel_z<32764) z=Accel_z/16384;
	else              z=(Accel_z-49152)/16384;

	//�ü��ٶȼ����������ˮƽ������ϵ֮��ļн�
	Angle_x_temp=(atan(y/z))*180/3.14;
	Angle_y_temp=(atan(x/z))*180/3.14;

	//�Ƕȵ�������											
	if(Accel_x<32764) Angle_y_temp = +Angle_y_temp;
	if(Accel_x>32764) Angle_y_temp = -Angle_y_temp;
	if(Accel_y<32764) Angle_x_temp = +Angle_x_temp;
	if(Accel_y>32764) Angle_x_temp = -Angle_x_temp;
	if(Accel_z<32764) {}
	if(Accel_z>32764) {}
	
	//���ٶ�
	//��ǰ�˶�
	if(Gyro_x<32768) Gyro_x=-(Gyro_x/16.4);//��ΧΪ1000deg/sʱ�������ϵ��16.4 LSB/(deg/s)
	//����˶�
	if(Gyro_x>32768) Gyro_x=+(65535-Gyro_x)/16.4;
	//��ǰ�˶�
	if(Gyro_y<32768) Gyro_y=-(Gyro_y/16.4);//��ΧΪ1000deg/sʱ�������ϵ��16.4 LSB/(deg/s)
	//����˶�
	if(Gyro_y>32768) Gyro_y=+(65535-Gyro_y)/16.4;
	//��ǰ�˶�
	if(Gyro_z<32768) Gyro_z=-(Gyro_z/16.4);//��ΧΪ1000deg/sʱ�������ϵ��16.4 LSB/(deg/s)
	//����˶�
	if(Gyro_z>32768) Gyro_z=+(65535-Gyro_z)/16.4;
	
	//Angle_gy = Angle_gy + Gyro_y*0.025;  //���ٶȻ��ֵõ���б�Ƕ�.Խ����ֳ����ĽǶ�Խ��
	Kalman_Filter_X(Angle_x_temp,Gyro_x);  //�������˲�����Y���
	Kalman_Filter_Y(Angle_y_temp,Gyro_y);  //�������˲�����Y���
															  
} 


//����������		
float Q_angle = 0.001;  
float Q_gyro  = 0.003;
float R_angle = 0.5;
float dt      = 0.01;//dtΪkalman�˲�������ʱ��;
char  C_0     = 1;
float Q_bias, Angle_err;
float PCt_0, PCt_1, E;
float K_0, K_1, t_0, t_1;
float Pdot[4] ={0,0,0,0};
float PP[2][2] = { { 1, 0 },{ 0, 1 } };

void Kalman_Filter_X(float Accel,float Gyro) //����������		
{
	Angle_X_Final += (Gyro - Q_bias) * dt; //�������
	
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-����������Э�����΢��

	Pdot[1]= -PP[1][1];
	Pdot[2]= -PP[1][1];
	Pdot[3]= Q_gyro;
	
	PP[0][0] += Pdot[0] * dt;   // Pk-����������Э����΢�ֵĻ���
	PP[0][1] += Pdot[1] * dt;   // =����������Э����
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - Angle_X_Final;	//zk-�������
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //����������Э����
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	Angle_X_Final += K_0 * Angle_err;	 //�������
	Q_bias        += K_1 * Angle_err;	 //�������
	Gyro_x         = Gyro - Q_bias;	 //���ֵ(�������)��΢��=���ٶ�
}

void Kalman_Filter_Y(float Accel,float Gyro) //����������		
{
	Angle_Y_Final += (Gyro - Q_bias) * dt; //�������
	
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-����������Э�����΢��

	Pdot[1]=- PP[1][1];
	Pdot[2]=- PP[1][1];
	Pdot[3]=Q_gyro;
	
	PP[0][0] += Pdot[0] * dt;   // Pk-����������Э����΢�ֵĻ���
	PP[0][1] += Pdot[1] * dt;   // =����������Э����
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - Angle_Y_Final;	//zk-�������
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //����������Э����
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	Angle_Y_Final	+= K_0 * Angle_err;	 //�������
	Q_bias	+= K_1 * Angle_err;	 //�������
	Gyro_y   = Gyro - Q_bias;	 //���ֵ(�������)��΢��=���ٶ�
}

