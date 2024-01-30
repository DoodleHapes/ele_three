#ifndef __KALMAN_H__
#define __KALMAN_H__

//typedef struct
//{
//    /*不用动*/
//    float LastP;//上次估算协方差
//    float Now_P;//当前估算协方差
//	  float B;
//    float out;//卡尔曼滤波器输出
//    float Kg;//卡尔曼增益
//	float Q;
//	float R;
//}Kalman_Typedef;
extern float klm_Now_P[2];
extern float klm_Kg[2];
extern float klm_out[2];
extern float klm_LastP[2];
extern float klm_Q[2];
extern float klm_R[2];
extern float klm_B[2];

void Kalman_Init(void);
void KalmanFilter(float input);

#endif

