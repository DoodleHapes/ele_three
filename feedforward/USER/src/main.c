/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897(已满)  三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		main
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ790875685)
 * @version    		查看doc内version文件 版本说明
 * @Software 		MDK FOR C251 V5.60
 * @Target core		STC32G12K128
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-12-18
 ********************************************************************************************************************/

#include "headfile.h"
#include "oled_menu.h"
#include "key.h"
#include "flash.h"
#include "pwm_control.h"
#include "encoder.h"
#include "pid_control.h"
#include "oled_menu.h"
#include "gyro.h"
#include "fuzzy_control.h"



/*
 * 系统频率，可查看board.h中的 FOSC 宏定义修改。
 * board.h文件中FOSC的值设置为0,则程序自动设置系统频率为33.1776MHZ
 * 在board_init中,已经将P54引脚设置为复位
 * 如果需要使用P54引脚,可以在board.c文件中的board_init()函数中删除SET_P54_RESRT即可
 */




void main()
{
	board_init();			// 初始化寄存器,勿删除此句代码。
	ctimer_count_init(SPEEDL_PLUSE);	//初始化定时器0作为外部计数
	ctimer_count_init(SPEEDR_PLUSE);	//初始化定时器1作为外部计数
	iap_init();
	//NVIC_SetPriority(UART1_IRQn, 3);   //把中断定为最高优先级
	oled_init();
	my_pwm_init();
	my_adc_init();
	//uart_init(UART_1, UART1_RX_P36, UART1_TX_P37, 115200, TIM_2);
	key_init(10);
	volt=adc_once(ADC_P14,ADC_10BIT)/80.47;
	ele_init();
		P72=0;
	//Kalman_Init();
      //卡尔曼滤波参数初始化
	//P72=0;
//  while(imu963ra_init())				//六轴陀螺仪初始化
//	{
//		delay_ms(500);
//	}

//	get_gyro_err();
	
	//flash_number_init();
	//flash_write();
	flash_read();        //读取flash的值
	fuzzy_init();
	pit_timer_ms(TIM_3, 5);   //用定时器3进行电感检测和编码器检测和pwm赋值
	pit_timer_ms(TIM_4, 5);    //按键检测

	
	// 此处编写用户代码(例如：外设初始化代码等)
    while(1)
	{
		//oled_p6x8str(64,4,"Running");
		
//		if(start_flag==0) //按键按下时刷新菜单，不按时刷新电感值
//		{
		
			oled_menu();

		//		}
//		else
//			oled_clear();
		
	
		//传感版测试
	  
//	  adc_test_oled();
//	  delay_ms(80);
//		oled_clear();
//		adc_test();

    //gyro测试
		//oled_gyro_test();
		


		
		

		
  }
}

