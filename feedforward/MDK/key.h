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

//旧车
//#define KEY_UP 		(P76)
//#define KEY_DOWN  (P44) //mid  
//#define KEY_LEFT  (P20)
//#define KEY_RIGHT (P21)  //xia
//#define KEY_MID   (P75)  //you
//#define KEY_1     (P66)
//#define KEY_2     (P40)



#define KEY_RELEASE_LEVEL           (1)                                 // 按键的默认状态 也就是按键释放状态的电平
#define KEY_MAX_SHOCK_PERIOD        (100       )                                 // 按键消抖检测时长 单位毫秒 低于这个时长的信号会被认为是杂波抖动
#define KEY_LONG_PRESS_PERIOD       (500     )                                 // 最小长按时长 单位毫秒 高于这个时长的信号会被认为是长按动作



extern uint16 key_state[7];
extern uint8 	key_number;




                                                            // 按键索引 对应上方定义的按键引脚个数 默认定义四个按键
typedef enum
{
    KEY_RELEASE,                                                                // 按键释放状态
    KEY_SHORT_PRESS,                                                            // 按键短按状态
    KEY_LONG_PRESS,                                                             // 按键长按状态
}key_state_enum;


void key_scanner(void);
uint8 gpio_get_level(uint8 i);
void key_init(uint32 period);



#endif