#ifndef __OLED_MENU_H__
#define __OLED_MENU_H__

typedef enum
{
	init,
	sort,
	PID,
	pwm_factor,
	roundabout_page,
	motion,
	ele_list,
	kalman,
}page_index_enum;



	extern uint8 start_flag;
extern uint8 clear_flag;
extern uint8 on_off_motion[5];
extern float volt;

void page_pid(void);
void oled_menu(void);
void page_init(void);




#endif