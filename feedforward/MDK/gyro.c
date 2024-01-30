#include "gyro.h"
#include "SEEKFREE_IMU963RA.h"
#include "flash.h"
#include "math.h"
#include "SEEKFREE_OLED.h"

//int16 gy_zero;
//int16 gx_zero;
float Angle_gyro=0;
float gz_zero=0;
float gx_acc;
float gy_acc;
float gz_acc;
float gz;
float gz_total;
float gz_filter;
float gz_acc_filter;
float gy_acc_filter;
float gx_acc_filter;




//�ڷ���֮ǰ��ƫ��ֵ���������

void the_KalmanFilter(float input)
{
		
		
    //Ԥ��Э����̣�kʱ��ϵͳ����Э���� = k-1ʱ�̵�ϵͳЭ���� + ��������Э����
    Klm.Now_P = Klm.LastP + Klm.Q;
    //���������淽�̣����������� = kʱ��ϵͳ����Э���� / ��kʱ��ϵͳ����Э���� + �۲�����Э���
    Klm.Kg = Klm.Now_P / (Klm.Now_P + Klm.R);
    //��������ֵ���̣�kʱ��״̬����������ֵ = ״̬������Ԥ��ֵ + ���������� * ������ֵ - ״̬������Ԥ��ֵ��
    Klm.out = Klm.out + Klm.Kg * (input -Klm.out)+Klm.B;//��Ϊ��һ�ε�Ԥ��ֵ������һ�ε����ֵ
    //����Э�����: ���ε�ϵͳЭ����� klm->LastP Ϊ��һ������׼����
    Klm.LastP = (1-Klm.Kg) * Klm.Now_P;	
}
void Kalman_Init(void)//�¶�klm_Q=0.01 klm_R=0.25
{
	Klm.Now_P=0;			//��ǰ����Э����
	Klm.out=0;				//�������˲������
	Klm.Kg=0;				//����������
	Klm.LastP=0;
	//0.85 0.01
}
void get_gyro_err(void)  //��һ�����õ���ʼerr
{
	uint8 i;
	float gz_zero_total=0;
	for(i=0;i<50;i++)
	{
		imu963ra_get_gyro();
		gz_zero_total+=imu963ra_gyro_transition(imu963ra_gyro_z);
	}   //�õ�ֵ
	//gy_zero=icm20602_gyro_y;
	//gx_zero=icm20602_gyro_x;
	gz_zero=gz_zero_total/50;    // �õ���/s	

	
}




void gyro_get_roll(void)
{
//	for(i=10-roll_bits_gyro;i>=1;i--)
//	{
//		gz[i-1]=gz[i];
//		//gz_acc[i-1]=gz_acc[i];
//		gy_acc[i-1]=gy_acc[i];
//		gx_acc[i-1]=gx_acc[i];
//	}
		imu963ra_get_gyro();
		imu963ra_get_acc();
	  gz=imu963ra_gyro_transition(imu963ra_gyro_z)-gz_zero;    //��ȥ��ʼ�ٽ�ֵ
		//gx_acc=imu963ra_acc_transition(imu963ra_acc_x);   //��λΪ���ٶȵ�λ
//		gy_acc=imu963ra_acc_transition(imu963ra_acc_y);
//		gz_acc=imu963ra_acc_transition(imu963ra_gyro_z);		
}
void gyro_filter_roll_mid(void)
{
	gz_filter=gz;   //ת��Ϊ�Ƕ�
	//gx_acc_filter=gx_acc;
	gy_acc_filter=gy_acc;
	gz_acc_filter=gz_acc;
//		imu660ra_get_gyro();
//		imu660ra_get_acc();
//	  gz_filter=imu660ra_gyro_transition(imu660ra_gyro_z)-gz_zero;    //��ȥ��ʼ�ٽ�ֵ
//		gx_acc_filter=imu660ra_acc_transition(imu660ra_acc_x);   //��λΪ���ٶȵ�λ
//		gy_acc_filter=imu660ra_acc_transition(imu660ra_acc_y);
}
void gyro_acc_filter(void)
{
	gyro_get_roll();
	//max_min_gyro_acc_init();
	//find_the_trash_gyro();
	gyro_filter_roll_mid();
}
/***�����˲��Ƕȼ���***/
float AngleGet(void)
{
        //float dt = 0.0001249;//Gy 2msʱ�����ϵ��
	float dt=0.005;
  float Angle;      
	float Angle_acc=0;
	double angle_ratio;//���ٶȱ�ֵ
         /***����Ϊ���ٶȼ�ȡ�����еõ��Ƕ�***/
				
	
				
	
        angle_ratio=((double)gy_acc_filter)/(gz_acc_filter+0.1);    //ÿ�������Ƿ��õ�λ�ò�ͬ����Ҫȷ��
     
				Angle_acc=(float)atan(angle_ratio)*57.29578049;//���ٶȼƵõ��Ľ�

	
	
        if(Angle_acc > 89)
          Angle_acc = 89;
        if(Angle_acc < -89)
          Angle_acc = -89;   //�������      
				Angle_gyro+=gz_filter * dt;   //�����Ҽ�
				Angle=Angle_gyro+(Angle_acc-Angle_gyro)*0.01;
				//oled_gyro_test(Angle_acc,Angle);    //����
				//oled_printf_float(60,6,Angle_acc,3,3);
				return Angle;
		//�൱��Angle = Angle*(1-0.00105) + Angle_acc*0.001
}
float Angel_Get_Kal(void)
{
	float dt=0.005;
	the_KalmanFilter(gz_filter);
	Angle_gyro+=Klm.out*dt;
	return Angle_gyro;
}
void oled_gyro_test(float Angle_acc,float Angle)
{
	oled_p6x8str(0,0,"Angel_gyro:");
	oled_p6x8str(0,1,"Angel_acc:");
	oled_p6x8str(0,2,"Angel:");
//	oled_printf_float(64,0,Angle_gyro,3,1);  //�õ��Ƕ�
//	oled_printf_float(64,1,Angle_acc,3,1);
//	oled_printf_float(64,2,Angle,3,1);
	oled_printf_float(64,0,Angle_gyro,4,4);  //�õ��Ƕ�
	oled_printf_float(64,1,Angle_acc,4,4);
	oled_printf_float(64,2,Angle,4,4);
	
}


//�������������һֱ�������ʶ����
//������ʱʹ������������


