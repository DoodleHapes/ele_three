#ifndef __KEY_H__
#define __KEY_H__

#include "zf_gpio.h"
#include "STC32Gxx.h"

#define KEY_UP 		(P21)	 
#define KEY_DOWN  (P76) 	
#define KEY_LEFT  (P20)
#define KEY_RIGHT (P75)  
#define KEY_MID   (P44)  
#define KEY_1     (P40)
#define KEY_2     (P66)

//�ɳ�
//#define KEY_UP 		(P76)
//#define KEY_DOWN  (P44) //mid  
//#define KEY_LEFT  (P20)
//#define KEY_RIGHT (P21)  //xia
//#define KEY_MID   (P75)  //you
//#define KEY_1     (P66)
//#define KEY_2     (P40)



#define KEY_RELEASE_LEVEL           (1)                                 // ������Ĭ��״̬ Ҳ���ǰ����ͷ�״̬�ĵ�ƽ
#define KEY_MAX_SHOCK_PERIOD        (100       )                                 // �����������ʱ�� ��λ���� �������ʱ�����źŻᱻ��Ϊ���Ӳ�����
#define KEY_LONG_PRESS_PERIOD       (500     )                                 // ��С����ʱ�� ��λ���� �������ʱ�����źŻᱻ��Ϊ�ǳ�������



extern uint16 key_state[7];
extern uint8 	key_number;




                                                            // �������� ��Ӧ�Ϸ�����İ������Ÿ��� Ĭ�϶����ĸ�����
typedef enum
{
    KEY_RELEASE,                                                                // �����ͷ�״̬
    KEY_SHORT_PRESS,                                                            // �����̰�״̬
    KEY_LONG_PRESS,                                                             // ��������״̬
}key_state_enum;


void key_scanner(void);
uint8 gpio_get_level(uint8 i);
void key_init(uint32 period);



#endif