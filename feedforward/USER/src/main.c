/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897(����)  ��Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		main
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ790875685)
 * @version    		�鿴doc��version�ļ� �汾˵��
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
 * ϵͳƵ�ʣ��ɲ鿴board.h�е� FOSC �궨���޸ġ�
 * board.h�ļ���FOSC��ֵ����Ϊ0,������Զ�����ϵͳƵ��Ϊ33.1776MHZ
 * ��board_init��,�Ѿ���P54��������Ϊ��λ
 * �����Ҫʹ��P54����,������board.c�ļ��е�board_init()������ɾ��SET_P54_RESRT����
 */




void main()
{
	board_init();			// ��ʼ���Ĵ���,��ɾ���˾���롣
	ctimer_count_init(SPEEDL_PLUSE);	//��ʼ����ʱ��0��Ϊ�ⲿ����
	ctimer_count_init(SPEEDR_PLUSE);	//��ʼ����ʱ��1��Ϊ�ⲿ����
	iap_init();
	//NVIC_SetPriority(UART1_IRQn, 3);   //���ж϶�Ϊ������ȼ�
	oled_init();
	my_pwm_init();
	my_adc_init();
	//uart_init(UART_1, UART1_RX_P36, UART1_TX_P37, 115200, TIM_2);
	key_init(10);
	volt=adc_once(ADC_P14,ADC_10BIT)/80.47;
	ele_init();
		P72=0;
	//Kalman_Init();
      //�������˲�������ʼ��
	//P72=0;
//  while(imu963ra_init())				//���������ǳ�ʼ��
//	{
//		delay_ms(500);
//	}

//	get_gyro_err();
	
	//flash_number_init();
	//flash_write();
	flash_read();        //��ȡflash��ֵ
	fuzzy_init();
	pit_timer_ms(TIM_3, 5);   //�ö�ʱ��3���е�м��ͱ���������pwm��ֵ
	pit_timer_ms(TIM_4, 5);    //�������

	
	// �˴���д�û�����(���磺�����ʼ�������)
    while(1)
	{
		//oled_p6x8str(64,4,"Running");
		
//		if(start_flag==0) //��������ʱˢ�²˵�������ʱˢ�µ��ֵ
//		{
		
			oled_menu();

		//		}
//		else
//			oled_clear();
		
	
		//���а����
	  
//	  adc_test_oled();
//	  delay_ms(80);
//		oled_clear();
//		adc_test();

    //gyro����
		//oled_gyro_test();
		


		
		

		
  }
}

