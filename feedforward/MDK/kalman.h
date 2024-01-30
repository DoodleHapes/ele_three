#ifndef __KALMAN_H__
#define __KALMAN_H__

//typedef struct
//{
//    /*���ö�*/
//    float LastP;//�ϴι���Э����
//    float Now_P;//��ǰ����Э����
//	  float B;
//    float out;//�������˲������
//    float Kg;//����������
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

