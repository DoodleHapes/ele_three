#ifndef __GYRO_H__
#define __GYRO_H__


void get_gyro_acc_init(void);
void gyro_acc_filter(void);
void get_gyro_err(void);
void oled_gyro_test(float Angle_acc,float Angle);
float AngleGet(void);
void Kalman_Init(void);
float Angel_Get_Kal(void);



#endif