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




//在发车之前把偏差值算出消除掉

void the_KalmanFilter(float input)
{
		
		
    //预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
    Klm.Now_P = Klm.LastP + Klm.Q;
    //卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
    Klm.Kg = Klm.Now_P / (Klm.Now_P + Klm.R);
    //更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
    Klm.out = Klm.out + Klm.Kg * (input -Klm.out)+Klm.B;//因为这一次的预测值就是上一次的输出值
    //更新协方差方程: 本次的系统协方差赋给 klm->LastP 为下一次运算准备。
    Klm.LastP = (1-Klm.Kg) * Klm.Now_P;	
}
void Kalman_Init(void)//温度klm_Q=0.01 klm_R=0.25
{
	Klm.Now_P=0;			//当前估算协方差
	Klm.out=0;				//卡尔曼滤波器输出
	Klm.Kg=0;				//卡尔曼增益
	Klm.LastP=0;
	//0.85 0.01
}
void get_gyro_err(void)  //按一个键得到初始err
{
	uint8 i;
	float gz_zero_total=0;
	for(i=0;i<50;i++)
	{
		imu963ra_get_gyro();
		gz_zero_total+=imu963ra_gyro_transition(imu963ra_gyro_z);
	}   //得到值
	//gy_zero=icm20602_gyro_y;
	//gx_zero=icm20602_gyro_x;
	gz_zero=gz_zero_total/50;    // 得到度/s	

	
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
	  gz=imu963ra_gyro_transition(imu963ra_gyro_z)-gz_zero;    //减去初始临界值
		//gx_acc=imu963ra_acc_transition(imu963ra_acc_x);   //单位为加速度单位
//		gy_acc=imu963ra_acc_transition(imu963ra_acc_y);
//		gz_acc=imu963ra_acc_transition(imu963ra_gyro_z);		
}
void gyro_filter_roll_mid(void)
{
	gz_filter=gz;   //转换为角度
	//gx_acc_filter=gx_acc;
	gy_acc_filter=gy_acc;
	gz_acc_filter=gz_acc;
//		imu660ra_get_gyro();
//		imu660ra_get_acc();
//	  gz_filter=imu660ra_gyro_transition(imu660ra_gyro_z)-gz_zero;    //减去初始临界值
//		gx_acc_filter=imu660ra_acc_transition(imu660ra_acc_x);   //单位为加速度单位
//		gy_acc_filter=imu660ra_acc_transition(imu660ra_acc_y);
}
void gyro_acc_filter(void)
{
	gyro_get_roll();
	//max_min_gyro_acc_init();
	//find_the_trash_gyro();
	gyro_filter_roll_mid();
}
/***互补滤波角度计算***/
float AngleGet(void)
{
        //float dt = 0.0001249;//Gy 2ms时间积分系数
	float dt=0.005;
  float Angle;      
	float Angle_acc=0;
	double angle_ratio;//加速度比值
         /***以下为加速度计取反正切得到角度***/
				
	
				
	
        angle_ratio=((double)gy_acc_filter)/(gz_acc_filter+0.1);    //每个陀螺仪放置的位置不同，需要确定
     
				Angle_acc=(float)atan(angle_ratio)*57.29578049;//加速度计得到的角

	
	
        if(Angle_acc > 89)
          Angle_acc = 89;
        if(Angle_acc < -89)
          Angle_acc = -89;   //左减右正      
				Angle_gyro+=gz_filter * dt;   //左正右减
				Angle=Angle_gyro+(Angle_acc-Angle_gyro)*0.01;
				//oled_gyro_test(Angle_acc,Angle);    //测验
				//oled_printf_float(60,6,Angle_acc,3,3);
				return Angle;
		//相当于Angle = Angle*(1-0.00105) + Angle_acc*0.001
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
//	oled_printf_float(64,0,Angle_gyro,3,1);  //得到角度
//	oled_printf_float(64,1,Angle_acc,3,1);
//	oled_printf_float(64,2,Angle,3,1);
	oled_printf_float(64,0,Angle_gyro,4,4);  //得到角度
	oled_printf_float(64,1,Angle_acc,4,4);
	oled_printf_float(64,2,Angle,4,4);
	
}


//在连续弯道，会一直处于弯道识别嘛
//来回震荡时使用陀螺仪作用


