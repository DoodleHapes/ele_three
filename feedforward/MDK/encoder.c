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
	//������Ϣ��Ҫ����
        tempr_pluse = ctimer_count_read(SPEEDL_PLUSE);
		templ_pluse = ctimer_count_read(SPEEDR_PLUSE);

        //����������
    ctimer_count_clean(SPEEDL_PLUSE);
		ctimer_count_clean(SPEEDR_PLUSE);

        //�ɼ�������Ϣ
        if(0 == SPEEDL_DIR)    
        {
            tempr_pluse = -tempr_pluse;
        }

		if(0 != SPEEDR_DIR)   //����˳��    
        {
            templ_pluse = -templ_pluse;
        }
        
}
void encoder_spd_get(void)
{
	encoder_handle();
	spd_l=templ_pluse*1.692;   //�õ�ת��,cm/s
	spd_r=tempr_pluse*1.692;
	//spd_l=(templ_pluse<<1)+templ_pluse*1.384;   //�õ�ת��,cm/s
	//spd_r=(tempr_pluse<<1)+tempr_pluse*1.384;
	
	
  //the_KalmanFilter((float)templ_pluse,(float)tempr_pluse);
//	klm_l.out=templ_pluse;
//	klm_r.out=tempr_pluse;
//	spd_l=klm_l.out*3.384;   //�õ�ת��,cm/s
//	spd_r=klm_r.out*3.384;
	//printf("%f,%f\n",spd_l,spd_r);    
}
