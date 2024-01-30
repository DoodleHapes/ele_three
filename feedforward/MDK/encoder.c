#include "encoder.h"
#include "common.h"
#include "zf_tim.h"
#include "kalman.h"
#include "flash.h"
#include "oled_menu.h"

int16 templ_pluse;    
int16 tempr_pluse;
float spd_l;
float spd_r;








void encoder_handle(void)
{	
	//方向信息需要调整
        tempr_pluse = ctimer_count_read(SPEEDL_PLUSE);
		templ_pluse = ctimer_count_read(SPEEDR_PLUSE);

        //计数器清零
    ctimer_count_clean(SPEEDL_PLUSE);
		ctimer_count_clean(SPEEDR_PLUSE);

        //采集方向信息
        if(0 == SPEEDL_DIR)    
        {
            tempr_pluse = -tempr_pluse;
        }

		if(0 != SPEEDR_DIR)   //调整顺序    
        {
            templ_pluse = -templ_pluse;
        }
        
}
void encoder_spd_get(void)
{
	encoder_handle();
	spd_l=templ_pluse*1.692;   //得到转速,cm/s
	spd_r=tempr_pluse*1.692;
	//spd_l=(templ_pluse<<1)+templ_pluse*1.384;   //得到转速,cm/s
	//spd_r=(tempr_pluse<<1)+tempr_pluse*1.384;
	
	
  //the_KalmanFilter((float)templ_pluse,(float)tempr_pluse);
//	klm_l.out=templ_pluse;
//	klm_r.out=tempr_pluse;
//	spd_l=klm_l.out*3.384;   //得到转速,cm/s
//	spd_r=klm_r.out*3.384;
	//printf("%f,%f\n",spd_l,spd_r);    
}
