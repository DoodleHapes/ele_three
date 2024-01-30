#ifndef __PID_CONTROL_H__
#define __PID_CONTROL_H__
#include "headfile.h"


extern float duty_left;
extern float duty_right;
extern int32 convert_l;
extern int32 convert_r;
extern float err_induct;
extern uint8 show_factor_flag;
extern uint8 round_flag;
extern float Angle;
extern float Angle_err;
extern float Last_Angle;
extern uint8 is_roundabout_flag;
extern float Finish_Angle;
//extern factor_judge_enum factor_flag;


typedef enum
{
    roll,
    stright,
		cross,
    roundabout,     //»·µº


}factor_judge_enum;
typedef enum
{
	pre_round,
	dif_in,
	auto_trail,
	gyro_stright,
	out_round,
	round_done,

}factor_round_enum;
static factor_judge_enum factor_flag;
static factor_round_enum round_state;
//typedef enum
//{
//	
//}
void factor_judgement(void);
void err_get(void);
void car_run(void);
void duty_get(void);
void protection(void);
void gyro_get(void);
void ele_init(void);
float god_Sqrt(float x);



#endif