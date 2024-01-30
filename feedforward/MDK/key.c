#include "key.h"
#include "SEEKFREE_OLED.h"
#include "headfile.h"

static uint32               scanner_period = 0;                                 // 按键的扫描周期
static uint32               key_press_time[7];                         // 按键信号持续时长

uint8 key_number=7;
uint32 interr_counter[7]={0};    //长短按识别
uint16 key_state[7]={0};      //按键个数可修改，上下左右中独立按键1独立按键2，0为初始，1为短按，2为长按
uint16 last_key_state[7]={0};


void key_init(uint32 period)    //中断的周期使用period
{
	uint8 i;
	scanner_period=period;
	for(i=0;i<key_number;i++)   //对按键状态进行初始化
	{
		key_state[i]=KEY_RELEASE;
		interr_counter[i]=0;
	}
	
}

uint8 gpio_get_level(uint8 i)   //通过i来直接用switch指向对应引脚
{
	uint8 gpio_level;
	switch(i)
	{
		case 0:
		{
			if(KEY_UP==1)
				
			gpio_level=1;
			else gpio_level=0;
			break;
			
		}
		case 1:
		{
			if(KEY_DOWN==1)
			
			gpio_level=1;
			else gpio_level=0;
			break;
		}
		case 2:
		{
			
			if(KEY_LEFT==1)
			gpio_level=1;
			else gpio_level=0;
			break;
		}
		case 3:
		{
			
			if(KEY_RIGHT==1)
			gpio_level=1;
			else gpio_level=0;
			break;
		}
		case 4:
		{
			
			if(KEY_MID==1)
			gpio_level=1;
			else gpio_level=0;
			break;
		}
		case 5:
		{
			
			if(KEY_1==1)
			gpio_level=1;
			else gpio_level=0;
			break;
		}
		case 6:
		
		{
			if(KEY_2==1)
			
			gpio_level=1;
			else gpio_level=0;
			break;
		}
		
		
			
	}
	
	
	return gpio_level;
}

void key_scanner(void)
{
	//有可能按1次会有增加好几个数字的效果，如果存在，就增加一个按键状态变化的标志位
	uint8 i;
	
	for(i=0;i<key_number;i++)   
	{
		if(KEY_RELEASE_LEVEL != gpio_get_level(i))       //当按下按键时，对按键进行宏定义，从而可以用i来进行区分
		{
			interr_counter[i]++;
		
				
	  if(interr_counter[i]*scanner_period>=KEY_LONG_PRESS_PERIOD)
		key_state[i]=KEY_LONG_PRESS;
		last_key_state[i]=KEY_LONG_PRESS;		
		
		}
		else    //按键释放
		{
            if((KEY_LONG_PRESS != key_state[i]) && (KEY_MAX_SHOCK_PERIOD <= scanner_period*interr_counter[i])&&last_key_state[i]!=KEY_SHORT_PRESS)
            {
              key_state[i] = KEY_SHORT_PRESS;     //只会执行一次
							last_key_state[i]=KEY_SHORT_PRESS;
							
						
							
							
            }
            else
            {
                key_state[i] = KEY_RELEASE;
							last_key_state[i]=KEY_RELEASE;
            }			
		
		interr_counter[i]=0;    //将按键计数请零
		}
	}	
}






