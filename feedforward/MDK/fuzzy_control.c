#include "fuzzy_control.h"
#include "common.h"
#include "flash.h"
#include "pid_control.h"

struct fuzzy_TypeDef fuzzy;
float final_p;  //���յ�p
float final_d;	//���յ�d



//struct fuzzy_TypeDef* fuzzy
//struct fuzzy_TypeDef fuzzy;
void fuzzy_init(void)
{
	fuzzy.p[0]=p_zero;
	fuzzy.p[1]=p_zero+delta_p_sm;
	fuzzy.p[2]=p_zero+delta_p_bst;
	fuzzy.d[0]=d_zero-delta_d_bst;
	fuzzy.d[1]=d_zero-delta_d_sm;
	fuzzy.d[2]=d_zero;
	fuzzy.d[3]=d_zero+delta_p_sm;
	fuzzy.d[4]=d_zero+delta_d_bst;
	
}
void degree_cal(float err,float d_err)   //err,d_err��Ҫ��һ���������
{

	uint8 k_z=2;
	uint8 k_s_up=2;
	uint8 k_s_down=4;
	uint8 k_b=2.67;
	memset(fuzzy.degree_mem_p,0,sizeof(fuzzy.degree_mem_p));
	memset(fuzzy.degree_mem_d,0,sizeof(fuzzy.degree_mem_d));//ÿ�μ���ǰ������������
		//�����ǻ����Լ����úõ������Ⱥ��������������
	if(err>=0)
	{
		if(err<=0.5)
	{
		fuzzy.degree_mem_p[2]=1-k_z*err;
	}
	if(err<=0.75)
	{
		fuzzy.degree_mem_p[3]=(err >= 0.5) ? (3-k_s_down*err) : (k_s_up*err);
	}
	if(err>=0.375)
	{
		fuzzy.degree_mem_p[4]=(err >= 0.75) ? 1 : (k_b*err-1);
	}
	}
	else
	{
		if(err>=-0.5)
		{
			fuzzy.degree_mem_p[2]=k_z*err+1;
		}
		if(err>=-0.75)
		{
			fuzzy.degree_mem_p[1] = (err >= -0.5) ? (-k_s_up*err) : (3+k_s_down*err);
		}
		if(err<=-0.375)
		{
			fuzzy.degree_mem_p[0] =(err >= -0.75) ? (-k_b*err-1) : 1;
		}	
	}
	if(d_err>=0)
	{	
		if(d_err<=0.5)
	{
		fuzzy.degree_mem_d[2]=1-k_z*d_err;
	}
	if(d_err<=0.75)
	{
		fuzzy.degree_mem_d[3]=(d_err >= 0.5) ? (3-k_s_down*d_err) : (k_s_up*d_err);
	}
	if(d_err>=0.375)
	{
		fuzzy.degree_mem_d[4]=(d_err >= 0.75) ? 1 : (k_b*d_err-1);	
	}
	}
	else
	{
		if(d_err>=-0.5)
		{
			fuzzy.degree_mem_d[2]=k_z*d_err+1;
		}
		if(d_err>=-0.75)
		{
			fuzzy.degree_mem_d[1] = (d_err >= -0.5) ? (-k_s_up*d_err) : (3+k_s_down*d_err);
		}
		if(d_err<=-0.375)
		{
			fuzzy.degree_mem_d[0] = (d_err >= -0.75) ? (-k_b*d_err-1) : 1;
		}
	}
}
float* mem_mutip(uint8 block)   //λ��ģ���������ϣ����ϣ����£������ĸ�����
{
	static float mem_mutip[9];  //�����ң����ϵ��£�3*3����
	switch(block)
	{
		case 0:
		{

			
			mem_mutip[0]=fuzzy.degree_mem_p[0]*fuzzy.degree_mem_d[0];
			mem_mutip[1]=fuzzy.degree_mem_p[1]*fuzzy.degree_mem_d[0];
			mem_mutip[2]=fuzzy.degree_mem_p[2]*fuzzy.degree_mem_d[0];
			mem_mutip[3]=fuzzy.degree_mem_p[0]*fuzzy.degree_mem_d[1];
			mem_mutip[4]=fuzzy.degree_mem_p[1]*fuzzy.degree_mem_d[1];
			mem_mutip[5]=fuzzy.degree_mem_p[2]*fuzzy.degree_mem_d[1];
			mem_mutip[6]=fuzzy.degree_mem_p[0]*fuzzy.degree_mem_d[2];
			mem_mutip[7]=fuzzy.degree_mem_p[1]*fuzzy.degree_mem_d[2];
			mem_mutip[8]=fuzzy.degree_mem_p[2]*fuzzy.degree_mem_d[2];
			break;
		}
		case 1:
		{//2
			
			mem_mutip[0]=fuzzy.degree_mem_p[2]*fuzzy.degree_mem_d[0];
			mem_mutip[1]=fuzzy.degree_mem_p[3]*fuzzy.degree_mem_d[0];
			mem_mutip[2]=fuzzy.degree_mem_p[4]*fuzzy.degree_mem_d[0];
			mem_mutip[3]=fuzzy.degree_mem_p[2]*fuzzy.degree_mem_d[1];
			mem_mutip[4]=fuzzy.degree_mem_p[3]*fuzzy.degree_mem_d[1];
			mem_mutip[5]=fuzzy.degree_mem_p[4]*fuzzy.degree_mem_d[1];
			mem_mutip[6]=fuzzy.degree_mem_p[2]*fuzzy.degree_mem_d[2];
			mem_mutip[7]=fuzzy.degree_mem_p[3]*fuzzy.degree_mem_d[2];
			mem_mutip[8]=fuzzy.degree_mem_p[4]*fuzzy.degree_mem_d[2];
			break;
		}
		case 2:
		{//1
			
			mem_mutip[0]=fuzzy.degree_mem_p[0]*fuzzy.degree_mem_d[2];
			mem_mutip[1]=fuzzy.degree_mem_p[1]*fuzzy.degree_mem_d[2];
			mem_mutip[2]=fuzzy.degree_mem_p[2]*fuzzy.degree_mem_d[2];
			mem_mutip[3]=fuzzy.degree_mem_p[0]*fuzzy.degree_mem_d[3];
			mem_mutip[4]=fuzzy.degree_mem_p[1]*fuzzy.degree_mem_d[3];
			mem_mutip[5]=fuzzy.degree_mem_p[2]*fuzzy.degree_mem_d[3];
			mem_mutip[6]=fuzzy.degree_mem_p[0]*fuzzy.degree_mem_d[4];
			mem_mutip[7]=fuzzy.degree_mem_p[1]*fuzzy.degree_mem_d[4];
			mem_mutip[8]=fuzzy.degree_mem_p[2]*fuzzy.degree_mem_d[4];
			break;
		}
		case 3:
		{
			
			mem_mutip[0]=fuzzy.degree_mem_p[2]*fuzzy.degree_mem_d[2];
			mem_mutip[1]=fuzzy.degree_mem_p[3]*fuzzy.degree_mem_d[2];
			mem_mutip[2]=fuzzy.degree_mem_p[4]*fuzzy.degree_mem_d[2];
			mem_mutip[3]=fuzzy.degree_mem_p[2]*fuzzy.degree_mem_d[3];
			mem_mutip[4]=fuzzy.degree_mem_p[3]*fuzzy.degree_mem_d[3];
			mem_mutip[5]=fuzzy.degree_mem_p[4]*fuzzy.degree_mem_d[3];
			mem_mutip[6]=fuzzy.degree_mem_p[2]*fuzzy.degree_mem_d[4];
			mem_mutip[7]=fuzzy.degree_mem_p[3]*fuzzy.degree_mem_d[4];
			mem_mutip[8]=fuzzy.degree_mem_p[4]*fuzzy.degree_mem_d[4];
			break;
		}
	}
	return mem_mutip;
}
void fuzzy_rule(float err,float d_err) //����õ����յ�p��dֵ
{
	uint8 i;
	float* mem_mutiple;
	
	float total_mem_p=0;
	float total_mem_d=0;
		
	for(i=0;i<5;i++)
	{
		total_mem_p+=fuzzy.degree_mem_p[i];
		total_mem_d+=fuzzy.degree_mem_d[i];
	}
	for(i=0;i<5;i++)
	{
		fuzzy.degree_mem_p[i]=fuzzy.degree_mem_p[i]/total_mem_p;
		fuzzy.degree_mem_d[i]=fuzzy.degree_mem_d[i]/total_mem_d;
	}  //��һ��
	if(err<=0&&d_err<=0)
	{	
		mem_mutiple=mem_mutip(0);
		final_p=fuzzy.p[2]*(mem_mutiple[0]+mem_mutiple[3]+mem_mutiple[6])+fuzzy.p[1]*(mem_mutiple[1]+mem_mutiple[4]+mem_mutiple[7])+fuzzy.p[0]*(mem_mutiple[2]+mem_mutiple[5]+mem_mutiple[8]);
		final_d=fuzzy.d[4]*(mem_mutiple[0]+mem_mutiple[1])+fuzzy.d[3]*(mem_mutiple[3]+mem_mutiple[4])+fuzzy.d[0]*mem_mutiple[2]+fuzzy.d[1]*mem_mutiple[5]+fuzzy.d[2]*(mem_mutiple[6]+mem_mutiple[7]+mem_mutiple[8]);
	}
	else if(err>=0&&d_err<=0)
	{
		mem_mutiple=mem_mutip(1);	
		final_p=fuzzy.p[0]*(mem_mutiple[0]+mem_mutiple[3]+mem_mutiple[6])+fuzzy.p[1]*(mem_mutiple[1]+mem_mutiple[4]+mem_mutiple[7])+fuzzy.p[2]*(mem_mutiple[2]+mem_mutiple[5]+mem_mutiple[8]);
		final_d=fuzzy.d[0]*(mem_mutiple[0]+mem_mutiple[2])+fuzzy.d[1]*(mem_mutiple[3]+mem_mutiple[5])+fuzzy.d[3]*mem_mutiple[4]+fuzzy.d[4]*mem_mutiple[1]+fuzzy.d[2]*(mem_mutiple[6]+mem_mutiple[7]+mem_mutiple[8]);
	}
	else if(err<=0&&d_err>=0)
	{
		mem_mutiple=mem_mutip(2);	
		final_p=fuzzy.p[2]*(mem_mutiple[0]+mem_mutiple[3]+mem_mutiple[6])+fuzzy.p[1]*(mem_mutiple[1]+mem_mutiple[4]+mem_mutiple[7])+fuzzy.p[0]*(mem_mutiple[2]+mem_mutiple[5]+mem_mutiple[8]);
		final_d=fuzzy.d[0]*(mem_mutiple[6]+mem_mutiple[8])+fuzzy.d[1]*(mem_mutiple[3]+mem_mutiple[5])+fuzzy.d[2]*(mem_mutiple[0]+mem_mutiple[1]+mem_mutiple[2])+fuzzy.d[3]*mem_mutiple[4]+fuzzy.d[4]*mem_mutiple[7];
	}
	else if(err>=0&&d_err>=0)
	{
		mem_mutiple=mem_mutip(3);	
		final_p=fuzzy.p[0]*(mem_mutiple[0]+mem_mutiple[3]+mem_mutiple[6])+fuzzy.p[1]*(mem_mutiple[1]+mem_mutiple[4]+mem_mutiple[7])+fuzzy.p[2]*(mem_mutiple[2]+mem_mutiple[5]+mem_mutiple[8]);
		final_d=fuzzy.d[0]*mem_mutiple[6]+fuzzy.d[1]*mem_mutiple[3]+fuzzy.d[2]*(mem_mutiple[0]+mem_mutiple[1]+mem_mutiple[2])+fuzzy.d[3]*(mem_mutiple[4]+mem_mutiple[5])+fuzzy.d[4]*(mem_mutiple[7]+mem_mutiple[8]);
	}
}
float uniform(float input,float input_max)
{
	float out_put;
	out_put=(input+input_max)/input_max-1;
	//���й�һ��������Ĭ����Сֵ�����ֵ���
	return out_put;
}
void fuzzy_pid(float err,float d_err)
{
	float actual_err;
	float actual_d_err;
	actual_err=uniform(err,max_err_bound);
	actual_d_err=uniform(d_err,max_d_err_bound);	
	degree_cal(actual_err,actual_d_err);
	fuzzy_rule(actual_err,actual_d_err);
}
