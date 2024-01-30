#include "pwm_control.h"
#include "zf_pwm.h"
#include "pid_control.h"
#include "oled_menu.h"
#include "flash.h"




#define PWM_LEFT  (PWMA_CH1N_P11)
#define PWM_LEFT_CONVERT (PWMA_CH2N_P13)
#define PWM_RIGHT  (PWMA_CH3N_P65)
#define PWM_RIGHT_CONVERT (PWMA_CH4N_P67)   //lianggebo

//旧车
//#define PWM_LEFT  (PWMA_CH3N_P65)
//#define PWM_LEFT_CONVERT (PWMA_CH4N_P67)
//#define PWM_RIGHT  (PWMA_CH1N_P11)
//#define PWM_RIGHT_CONVERT (PWMA_CH2N_P13)   //lianggebo

void my_pwm_init(void)
{
	pwm_init(PWM_LEFT,17000, 0);
	pwm_init(PWM_RIGHT,17000,0);
	pwm_init(PWM_LEFT_CONVERT,17000,0);
	pwm_init(PWM_RIGHT_CONVERT,17000,0);
}
void pwm_control(void)
{
	if(start_flag==1)
	{
		if(duty_left<=0&&duty_right>0)
	{
		pwm_duty(PWM_LEFT,0);
		pwm_duty(PWM_RIGHT,(uint32)duty_right);
		pwm_duty(PWM_LEFT_CONVERT,(uint32)convert_l);
		pwm_duty(PWM_RIGHT_CONVERT,0);
	}
	else if(duty_right<=0&&duty_left>0)
	{
		pwm_duty(PWM_LEFT,(uint32)duty_left);
		pwm_duty(PWM_RIGHT,0);
		pwm_duty(PWM_LEFT_CONVERT,0);
		pwm_duty(PWM_RIGHT_CONVERT,(uint32)convert_r);
	}
	else if(duty_right<=0&&duty_left<=0)
	{
		pwm_duty(PWM_LEFT,0);
		pwm_duty(PWM_RIGHT,0);
		pwm_duty(PWM_LEFT_CONVERT,(uint32)convert_l);
		pwm_duty(PWM_RIGHT_CONVERT,(uint32)convert_r);	
	
	}
	else
	{
		pwm_duty(PWM_LEFT,(uint32)duty_left);
		pwm_duty(PWM_RIGHT,(uint32)duty_right);
		pwm_duty(PWM_LEFT_CONVERT,0);
		pwm_duty(PWM_RIGHT_CONVERT,0);
	}
	}
	else   //触发保护
	{
		pwm_duty(PWM_LEFT,0);  //yz
		pwm_duty(PWM_RIGHT,0);    //zz
		pwm_duty(PWM_LEFT_CONVERT,0); //yf
		pwm_duty(PWM_RIGHT_CONVERT,0);		
	}
}