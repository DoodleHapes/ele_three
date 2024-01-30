#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "common.h"


//定义脉冲引脚
#define SPEEDR_PLUSE   CTIM0_P34
#define SPEEDL_PLUSE   CTIM1_P35
//定义方向引脚
#define SPEEDR_DIR     P32
#define SPEEDL_DIR     P51


extern int16 templ_pluse;    
extern int16 tempr_pluse;
extern float spd_l;
extern float spd_r;
void encoder_handle(void);
void encoder_spd_get(void);


#endif