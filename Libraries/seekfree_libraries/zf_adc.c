/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897(����)  ��Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		adc
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ790875685)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		MDK FOR C251 V5.60
 * @Target core		STC32G12K128
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-4-14
 ********************************************************************************************************************/


#include "zf_adc.h"
#include "intrins.h"
#include "SEEKFREE_OLED.h"





#define lef_induct (ADC_P10)
#define m_lef_induct (ADC_P06)
#define m_rig_induct  (ADC_P04)
#define rig_induct 	(ADC_P02)
#define frd_lef_induct 	(ADC_P15)
#define frd_rig_induct 	(ADC_P01)

float induct1_filter;
float induct2_filter;
float induct3_filter;
float induct4_filter;
float induct5_filter;
float induct6_filter;
//float induct7_filter;
//float induct8_filter;


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ADC��ʼ��
//  @param      adcn            ѡ��ADCͨ��
//  @param      speed      		ADCʱ��Ƶ��
//  @return     void
//  Sample usage:               adc_init(ADC_P10,ADC_SYSclk_DIV_2);//��ʼ��P1.0ΪADC����,ADCʱ��Ƶ�ʣ�SYSclk/2
//-------------------------------------------------------------------------------------------------------------------
void adc_init(ADCN_enum adcn,ADC_SPEED_enum speed)
{
	ADC_CONTR |= 1<<7;				//1 ���� ADC ��Դ
	
	ADC_CONTR &= (0xF0);			//���ADC_CHS[3:0] �� ADC ģ��ͨ��ѡ��λ
	ADC_CONTR |= adcn;
	
	if((adcn >> 3) == 1) //P0.0
	{
		//IO����Ҫ����Ϊ��������
		P0M0 &= ~(1 << (adcn & 0x07));
		P0M1 |= (1 << (adcn & 0x07));
	}
	else if((adcn >> 3) == 0) //P1.0	
	{
		//IO����Ҫ����Ϊ��������
		P1M0 &= ~(1 << (adcn & 0x07));
	    P1M1 |= (1 << (adcn & 0x07));
	}

	ADCCFG |= speed&0x0F;			//ADCʱ��Ƶ��SYSclk/2/speed&0x0F;
	
	ADCCFG |= 1<<5;					//ת������Ҷ��롣 ADC_RES �������ĸ� 2 λ�� ADC_RESL �������ĵ� 8 λ��
}



//-------------------------------------------------------------------------------------------------------------------
//  @brief      ADCת��һ��
//  @param      adcn            ѡ��ADCͨ��
//  @param      resolution      �ֱ���
//  @return     void
//  Sample usage:               adc_convert(ADC_P10, ADC_10BIT);
//-------------------------------------------------------------------------------------------------------------------
uint16 adc_once(ADCN_enum adcn,ADCRES_enum resolution)
{
	uint16 adc_value;
	
	ADC_CONTR &= (0xF0);			//���ADC_CHS[3:0] �� ADC ģ��ͨ��ѡ��λ
	ADC_CONTR |= adcn;
	
	ADC_CONTR |= 0x40;  			// ���� AD ת��
	while (!(ADC_CONTR & 0x20));  	// ��ѯ ADC ��ɱ�־
	ADC_CONTR &= ~0x20;  			// ����ɱ�־
	
	adc_value = ADC_RES;  			//�洢 ADC �� 12 λ����ĸ� 4 λ
	adc_value <<= 8;
	adc_value |= ADC_RESL;  		//�洢 ADC �� 12 λ����ĵ� 8 λ
	
	ADC_RES = 0;
	ADC_RESL = 0;
	
	adc_value >>= resolution;		//ȡ����λ
	

	return adc_value;
}
uint16 get_the_total(uint16* induct)
{
	uint16 total=0;
	uint8 i;
	for(i=0;i<10;i++)
	total+=induct[i];
	return total;
}
void rank(uint16 *idt,uint8 bits)
{
	uint8 j;
	uint8 m;
	uint16 temp;
		for(j=0;j<bits;j++)
		{
			
			for(m=j;m<=bits-j-1;m++)
			{
				if(idt[m]>idt[m+1])
				{
					temp=idt[m+1];
					idt[m+1]=idt[m];
					idt[m]=temp;
				}
			}	
		}
}
void adc_get_mid(void)   //��ֵ�˲�
{
	uint8 i;
	uint16 idt1[11];
	uint16 idt2[11];
	uint16 idt3[11];
	uint16 idt4[11];
	uint16 idt5[11];
	uint16 idt6[11];
	uint8 mid_value=5;
	for(i=0;i<11;i++)
	{	
			idt1[i]=adc_once(lef_induct,ADC_10BIT);
			idt2[i]=adc_once(m_lef_induct,ADC_10BIT);
			idt3[i]=adc_once(m_rig_induct,ADC_10BIT);
			idt4[i]=adc_once(rig_induct,ADC_10BIT);
			idt5[i]=adc_once(frd_lef_induct,ADC_10BIT);
			idt6[i]=adc_once(frd_rig_induct,ADC_10BIT);
			


}
		//������������
		rank(idt1,11);
		rank(idt2,11);
		rank(idt3,11);
		rank(idt4,11);
		rank(idt5,11);
		rank(idt6,11);
	
	induct1_filter=idt1[mid_value];
	induct2_filter=idt2[mid_value];
	induct3_filter=idt3[mid_value];
	induct4_filter=idt4[mid_value];
	induct5_filter=idt5[mid_value];
	induct6_filter=idt6[mid_value];
	
}
void my_adc_init(void)
{
	adc_init(ADC_P10,ADC_SYSclk_DIV_2);////
	adc_init(ADC_P15,ADC_SYSclk_DIV_2);////
	adc_init(ADC_P01,ADC_SYSclk_DIV_2);////
	adc_init(ADC_P02,ADC_SYSclk_DIV_2);////
	adc_init(ADC_P03,ADC_SYSclk_DIV_2);////
	adc_init(ADC_P04,ADC_SYSclk_DIV_2);////
	adc_init(ADC_P05,ADC_SYSclk_DIV_2);////
	adc_init(ADC_P06,ADC_SYSclk_DIV_2);////
	adc_init(ADC_P14,ADC_SYSclk_DIV_2);
}