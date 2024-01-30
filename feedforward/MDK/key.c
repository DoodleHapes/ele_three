#include "key.h"
#include "SEEKFREE_OLED.h"
#include "headfile.h"

static uint32               scanner_period = 0;                                 // ������ɨ������
static uint32               key_press_time[7];                         // �����źų���ʱ��

uint8 key_number=7;
uint32 interr_counter[7]={0};    //���̰�ʶ��
uint16 key_state[7]={0};      //�����������޸ģ����������ж�������1��������2��0Ϊ��ʼ��1Ϊ�̰���2Ϊ����
uint16 last_key_state[7]={0};


void key_init(uint32 period)    //�жϵ�����ʹ��period
{
	uint8 i;
	scanner_period=period;
	for(i=0;i<key_number;i++)   //�԰���״̬���г�ʼ��
	{
		key_state[i]=KEY_RELEASE;
		interr_counter[i]=0;
	}
	
}

uint8 gpio_get_level(uint8 i)   //ͨ��i��ֱ����switchָ���Ӧ����
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
	//�п��ܰ�1�λ������Ӻü������ֵ�Ч����������ڣ�������һ������״̬�仯�ı�־λ
	uint8 i;
	
	for(i=0;i<key_number;i++)   
	{
		if(KEY_RELEASE_LEVEL != gpio_get_level(i))       //�����°���ʱ���԰������к궨�壬�Ӷ�������i����������
		{
			interr_counter[i]++;
		
				
	  if(interr_counter[i]*scanner_period>=KEY_LONG_PRESS_PERIOD)
		key_state[i]=KEY_LONG_PRESS;
		last_key_state[i]=KEY_LONG_PRESS;		
		
		}
		else    //�����ͷ�
		{
            if((KEY_LONG_PRESS != key_state[i]) && (KEY_MAX_SHOCK_PERIOD <= scanner_period*interr_counter[i])&&last_key_state[i]!=KEY_SHORT_PRESS)
            {
              key_state[i] = KEY_SHORT_PRESS;     //ֻ��ִ��һ��
							last_key_state[i]=KEY_SHORT_PRESS;
							
						
							
							
            }
            else
            {
                key_state[i] = KEY_RELEASE;
							last_key_state[i]=KEY_RELEASE;
            }			
		
		interr_counter[i]=0;    //��������������
		}
	}	
}






