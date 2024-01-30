#ifndef __FLASH_H__
#define __FLASH_H__
#include "common.h"
extern uint8 on_off_motion[5];
extern uint8 ele[6];
extern struct flash_number flash;
extern struct roudabout round;
extern struct Kalman_Typedef Klm;
struct flash_number
{
    /*不用动*/
    int16 p_zero;//上次估算协方差
    int16 d_zero;//当前估算协方差
		int16 delta_p_sm;
		int16 delta_p_bst;
    int16 delta_d_sm;//卡尔曼滤波器输出
    int16 delta_d_bst;//卡尔曼增益
		int16 spd_kp;
		int16	spd_ki;
		float	max_err_bound;
		float	max_d_err_bound;
		int16 str_spd;
		int16 roll_spd;
		float str_roll_judge;
		uint8 cross_idt;
		
		
};
struct roudabout
{
	int16 gyro_roll;
	float at_trace_pro;
	float out_pro;
	int16 integral_in;
	int16 integral_out;
	int16 roud_in_dif;
	int16 roud_at_p;
	int16 roud_at_i;
	int16 roud_gy_p;
	int16 roud_gy_d;
	int16 brim_int;
	int16 mid_int;
	int16 roud_spd;
	int16 int_p;
	int16 int_d;
};
struct Kalman_Typedef
{
    /*不用动*/
    float LastP;//上次估算协方差
    float Now_P;//当前估算协方差
    float out;//卡尔曼滤波器输出
    float Kg;//卡尔曼增益
	  float Q;
	  float R;
		float B;
};

//PID参数
#define p_zero   (flash.p_zero)
#define d_zero   (flash.d_zero)
#define delta_p_sm  (flash.delta_p_sm)
#define delta_p_bst  (flash.delta_p_bst)
#define delta_d_sm   (flash.delta_d_sm)
#define delta_d_bst    (flash.delta_d_bst)
#define spd_kp    (flash.spd_kp)
#define spd_ki    (flash.spd_ki)




//其余参数
#define max_err_bound  (flash.max_err_bound)
#define max_d_err_bound  (flash.max_d_err_bound)
#define str_spd   (flash.str_spd)
#define roll_spd  (flash.roll_spd)
#define cross_idt  (flash.cross_idt)
#define str_roll_judge (flash.str_roll_judge)
#define gyro_motion    (on_off_motion[0])
#define idt_t_motion (on_off_motion[1])
#define feed_forward (on_off_motion[2])
#define gy_rd_motion (on_off_motion[3])
#define gy_bl_motion (on_off_motion[4])
#define gyro_roll     (round.gyro_roll)
#define at_trace_pro  (round.at_trace_pro)
#define out_pro       (round.out_pro)
#define integral_in   (round.integral_in)
#define integral_out  (round.integral_out)
#define roud_in_dif   (round.roud_in_dif)
#define roud_at_p    (round.roud_at_p)   
#define roud_at_i    (round.roud_at_i)
#define roud_gy_p     (round.roud_gy_p)
#define roud_gy_d  			(round.roud_gy_d)
#define brim_int    (round.brim_int)
#define mid_int     (round.mid_int)
#define roud_spd    (round.roud_spd)
#define int_p       (round.int_p)
#define int_d       (round.int_d)
#define klm_R         (Klm.R)
#define klm_Q         (Klm.Q)
#define klm_B 				(Klm.B)
#define ele_dir    				(ele[0])
#define ele_num 			(ele[1])
#define ele_Fir     	(ele[2])
#define ele_Sec 			(ele[3])
#define ele_Thr				(ele[4])
#define ele_For  			(ele[5])
#define ele_Fiv				(ele[6])



void flash_write(void);
void flash_read(void);
void flash_number_init(void);


#endif