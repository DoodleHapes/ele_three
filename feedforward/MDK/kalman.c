/*
卡尔曼滤波器
整理By 乙酸氧铍
*/
#include "kalman.h"
float klm_Now_P[2];
float klm_Kg[2];
float klm_out[2];
float klm_LastP[2];
float klm_Q[2];
float klm_R[2];
float klm_B[2];
void KalmanFilter(float input)
{
		
    //预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
    klm_Now_P = klm_LastP + klm_Q;
    //卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
    klm_Kg = klm_Now_P / (klm_Now_P + klm_R);
		//更新卡尔曼输出
		klm_out=klm_out+klm_B;
    //更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
    klm_out = klm_out + klm_Kg * (input -klm_out);//因为这一次的预测值就是上一次的输出值
    //更新协方差方程: 本次的系统协方差赋给 klm->LastP 为下一次运算准备。
    klm_LastP = (1-klm_Kg) * klm_Now_P;


}

void Kalman_Init(void)//温度klm_Q=0.01 klm_R=0.25
{
	//0代表左，1代表右
	klm_LastP[0]=0.02;		//上次估算协方差(在调节噪声Q之前，要给一个初值，在开始调节Q之后可以将此初值赋为0看看效果)
	klm_Now_P[0]=0;			//当前估算协方差
	klm_B[0]=0;         //过程噪声期望（最后调,值应该相当的小甚至为0）
	klm_out[0]=0;				//卡尔曼滤波器输出
	klm_Kg[0]=0;				//卡尔曼增益
	klm_Q[0]=0;			//Q:过程噪声协方差 Q参数调滤波后的曲线平滑程度，Q越小越平滑; （次调）
	klm_R[0]=0;			//R:观测噪声协方差 R参数调整滤波后的曲线与实测曲线的相近程度，R越小越接近(收敛越快）最先调


	klm_LastP[1]=0.02;		//上次估算协方差(在调节噪声Q之前，要给一个初值，在开始调节Q之后可以将此初值赋为0看看效果)
	klm_Now_P[1]=0;			//当前估算协方差
	klm_B[1]=0;         //过程噪声期望（最后调,值应该相当的小甚至为0）
	klm_out[1]=0;				//卡尔曼滤波器输出
	klm_Kg[1]=0;				//卡尔曼增益
	klm_Q[1]=0;			//Q:过程噪声协方差 Q参数调滤波后的曲线平滑程度，Q越小越平滑; （次调）
	klm_R[1]=0;			//R:观测噪声协方差 R参数调整滤波后的曲线与实测曲线的相近程度，R越小越接近(收敛越快）最先调	
}


