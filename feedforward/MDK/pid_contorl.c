#include "pid_control.h"
#include "math.h"
#include "common.h"
#include "zf_adc.h"
#include "flash.h"
#include "oled_menu.h"
#include "encoder.h"
#include "gyro.h"
#include "fuzzy_control.h"

float encoder_integral_in=0;
float encoder_integral_out=0;
float last_spd_out_l=0;
float last_spd_out_r=0;
float last_err_spd_r=0;
float last_err_spd_l=0;
float err_spd_r=0;
float err_spd_l=0;
float gyro_out_pos=0;
float pos_pid_out=0;
float init_Angle=0;
float Finish_Angle=0;

//��Ҫ������
//��ת�������״ﵽ�޷������¹켣���ȶ�
float err_induct;
float last_err_induct;
float d_err_induct;

uint8 is_roundabout_flag=0;
float Angle=0;
float Angle_err=0;
float Last_Angle=0;

float Last_Angle_err=0;
//duty
float duty_left=0;
float duty_right=0;
int32 convert_l=0;
int32 convert_r=0;



uint8 ele_count=0;


//ƽ����
float god_Sqrt(float x)
{
	
    float xhalf = 0.5f * x;
    int32 i = *(int32*)&x;
    i = 0x5f3759df - (i >> 1);
    x = *(float*)&i;
    x = x * (1.5f - xhalf * x * x);
	//�����ڴ�Ӧ�ù�����
    //x = x * (1.5f - xhalf * x * x);
    //x = x * (1.5f - xhalf * x * x);

    return 1 / x;
}
void err_get(void)
{
	last_err_induct=err_induct;
	//err_induct>0ʱ��С������ƫ
	if((induct2_filter+induct3_filter)!=0)
	{

		//err_induct=((induct1_filter-induct4_filter)*3000)/pow((induct2_filter+induct3_filter),1.8);
		//���Ų�Ⱥϣ�����������б��������,mid600,brim270
		err_induct=((god_Sqrt(induct1_filter)-god_Sqrt(induct4_filter))*1000)/(induct2_filter+induct3_filter);
		//
		//err_induct=(induct1_filter-induct4_filter)/(induct2_filter+induct3_filter)*(induct2_filter+induct3_filter);
		
			
	}
	else
		err_induct=0;
	if(err_induct>max_err_bound)
		err_induct=max_err_bound;
	else if(err_induct<-max_err_bound)
		err_induct=-max_err_bound;
	
	
	d_err_induct=err_induct-last_err_induct;
	
//	if(d_err_induct>max_d_err_bound)
//		err_induct=max_d_err_bound;
//	else if(d_err_induct<-max_d_err_bound)
//		err_induct=-max_d_err_bound;	
//	
//	if(d_err_induct>max_d_err)
//		max_d_err=d_err_induct;
//	if(max_d_err>6)
//		max_d_err=0;
	
	fuzzy_pid(err_induct,d_err_induct);

	//printf("%f,%f\n",induct1_filter,induct4_filter);
}
void ele_init(void)
{
	if(ele_dir==0)
		ele_count=0;
	else
		ele_count=ele_num-1;
}



//roundabout

uint8 roudabout_judge(void)
{
	uint8 roudabout_flag=0;
	if(induct2_filter>=mid_int&&induct3_filter>=mid_int&&induct1_filter>=brim_int&&induct4_filter>=brim_int)  //�����ٿ�err_induct����
	{
		roudabout_flag=1;    //������Ԫ����1
		if(is_roundabout_flag==0)
			is_roundabout_flag=1;
	}
	
	return roudabout_flag;
}  //�ǻ���return 1
void factor_judgement(void)
{
	static factor_judge_enum last_flag;
	last_flag=factor_flag;

	if(ele[ele_count+2]!=0&&(roudabout_judge()||is_roundabout_flag==1))  //�������ж�Ϊ2״̬ʱ��Ϊ��������ʱ����ֱ������ж�
	{
		factor_flag=roundabout;    //�������ж���㲹��
		if(last_flag!=roundabout)   //��һ���ж�Ϊ����
		{
			init_Angle=Angle;       //��¼��ǰ�����ǵ�ֵ
		if(ele[ele_count+2]==1)   //�����Ϊ��ת
		Finish_Angle=Angle+gyro_roll;  //����Բ��ʱ�ĽǶ�
		else   //��תʱ����
	  Finish_Angle=Angle-gyro_roll;
		Angle_err=0;    //��Angel_err����
		

			
		}

	}
	else if(factor_flag==cross||(induct5_filter>=cross_idt&&induct6_filter>=cross_idt))  //��ʱ�����������������ݲ�������ǰհԤ�⹤��
	{
		factor_flag=cross;
		if(err_induct>-str_roll_judge||err_induct<str_roll_judge)  //�ж�Ϊʮ�ֺ��ĵ�����Ϊֱ��ʱ�ٻָ�����ѭ��
			factor_flag=stright;
	}
	else if(err_induct<-str_roll_judge||err_induct>str_roll_judge)
	{
		factor_flag=roll;
	}
	else 
	{
		factor_flag=stright;
       		
	}
	
}
void pos_control(void)
{
		

    if(factor_flag!=roundabout)
	{
		//��ת�Ƕȼ�С
		pos_pid_out=final_p*err_induct+final_d*(err_induct-last_err_induct);	
	}
	else
		pos_pid_out=int_p*err_induct+int_d*(err_induct-last_err_induct);

	

}

void speed_control(uint8 flag)
{
	  float set_spd_l=0;
	  float set_spd_r=0;
  //ֱ�����������ν����Ϊֱ�������λ�û����Ѿ���������
		last_err_spd_r=err_spd_r;
		last_err_spd_l=err_spd_l;
	
		switch(flag)    //ֱ�����
		{
			case 0:   //���
			{
				set_spd_r=roll_spd+pos_pid_out;  //���趨�ٶ��ϵ�ƫ��
				set_spd_l=roll_spd-pos_pid_out;    //���ձ����õ�ת�ٽ��������ӵ������ٶ�
				err_spd_l=set_spd_l-spd_l;   //��
				err_spd_r=set_spd_r-spd_r;   //��
				break;
			}
			case 1:  //ֱ��
			{
			if(factor_flag!=roundabout)
			{
				set_spd_l=str_spd-pos_pid_out;
				set_spd_r=str_spd+pos_pid_out;
			}
			else   //ʹ�ò�������ٶ�Ӱ��
			{
				if(round_state!=out_round)    //����
				{set_spd_l=roud_spd;   //���Ա߾Ͳ��õ�����淶�켣��
				set_spd_r=roud_spd;}
				else
				{set_spd_l=str_spd;
				set_spd_r=str_spd;
				}
			
			}
				err_spd_l=set_spd_l-spd_l;  
				err_spd_r=set_spd_r-spd_r; 				
				break;
			}
			case 2:       //�����뻷
			{
			if((int32)ele[ele_count+2]==1)   //���뻷
			{
				set_spd_l=roud_spd-roud_in_dif;
				set_spd_r=roud_spd+roud_in_dif;
				err_spd_l=set_spd_l-spd_l;
				err_spd_r=set_spd_r-spd_r;
			}
			else if((int32)ele[ele_count+2]==2)
			{
				set_spd_l=roud_spd+roud_in_dif;
				set_spd_r=roud_spd-roud_in_dif;
				err_spd_l=set_spd_l-spd_l;
				err_spd_r=set_spd_r-spd_r;
			}
				break;
			}
			case 3:    //��������
			{
				set_spd_l=roud_spd+gyro_out_pos;
				set_spd_r=roud_spd-gyro_out_pos;
				err_spd_l=set_spd_l-spd_l;
				err_spd_r=set_spd_r-spd_r;
				break;
			}
			case 4:		//����ѭ��
			{
			if(pos_pid_out>0)   //��ת
			{
				set_spd_r=roud_spd+pos_pid_out;  //���趨�ٶ��ϵ�ƫ��
				set_spd_l=roud_spd-pos_pid_out;    //���ձ����õ�ת�ٽ��������ӵ������ٶ�
				err_spd_l=set_spd_l-spd_l;   //��
				err_spd_r=set_spd_r-spd_r;   //��
			}
			else //��ת
			{	
				set_spd_l=roud_spd-pos_pid_out;
				set_spd_r=roud_spd+pos_pid_out;
				err_spd_l=set_spd_l-spd_l;  
				err_spd_r=set_spd_r-spd_r;   
			}				
				break;
			}
		}

}

	
	
void duty_get_close(uint8 flag)
{
	//pd����ǰ����С�����޲�ʹ�õ�е�ֱ���ж���ʹ��һ���µ��ж�����
	//�˴���ȷ���Ƿ�Ҫʹ������pid������ʵ��Ч��
	//��֪���Ƿ�Ҫ��һ����������ڲ���Ӱ��
	//�������Ƿ�Ҫ��������pid

		if(start_flag==1)
	{
		last_spd_out_l=duty_left;
		last_spd_out_r=duty_right;
	}
	else
	{
		last_spd_out_l=0;
		last_spd_out_r=0;	
	}

	if(flag!=2)
	{

			duty_left=spd_ki*(err_spd_l)+spd_kp*(err_spd_l-last_err_spd_l)+last_spd_out_l;
			duty_right=spd_ki*(err_spd_r)+spd_kp*(err_spd_r-last_err_spd_r)+last_spd_out_r;			
		
	}

	else   //�������ж�
	{
		duty_left=roud_at_p*(err_spd_l-last_err_spd_l)+roud_at_i*(err_spd_l)+last_spd_out_l;
		duty_right=roud_at_p*(err_spd_r-last_err_spd_r)+roud_at_i*(err_spd_r)+last_spd_out_r;
	}


	
	
	//�����ǵ�ʹ��
	if(duty_left>9900)
		duty_left=9900;
  if(duty_right>9900)
		duty_right=9900;
	if(duty_left<=0)
	{
		
		convert_l=abs(duty_left);
		if(convert_l>=9900)
		{
			convert_l=9900;
			duty_left=-9900;
		}
	}
	else
		convert_l=0;
	if(duty_right<=0)
	{
		convert_r=abs(duty_right);
		if(convert_r>=9900)
		{
			convert_r=9900;
			duty_right=-9900;
		}
	}
	else 
		convert_r=0;

}
void gyro_handle(uint8 flag)
{
		//angle err������
		//Last_Angle_err=Angle_err;  //����ֻ��Angle��last_Angle,����Angle_err�����¸�ֵ���������߼���
		if(flag==0)
		{
			Angle_err=Angle-(init_Angle+at_trace_pro*(Finish_Angle-init_Angle));   //���Ҹ���,��0.5���ṩ���
		}
		else
		{
			Angle_err=Angle-Finish_Angle;        //����ʱҪ�����̬�����ĽǶ�   
		}
		//Ϊ�˱������һ������ʹ��
		
}

void encoder_integral_count(void)
{
	if(round_state==pre_round)
	{
		if((int32)ele[ele_count+2]==1)
		encoder_integral_in+=spd_l;     //ֱ��ͨ�����������������
	  else
		encoder_integral_in+=spd_r;
	}
	else if(round_state==out_round)
	{
		if((int32)ele[ele_count+2]==1)
		encoder_integral_out+=spd_l;     //ֱ��ͨ�����������������
	  else
		encoder_integral_out+=spd_r;	
	}
//  else
//	encoder_integral=0;        //���ü���ʱ������
	else if(round_state==round_done)
	{
		encoder_integral_in=0;
		encoder_integral_out=0;
	}
	
}
void duty_get_roundabout(void)
{
	//��ʼʹ��������ѭ��	
		switch(round_state)
		{
			case pre_round:    //Ĭ��ֱ�� 
			{
				pos_control();
				speed_control(1);
				
				duty_get_close(0);   //��ǰʹ�û����ٶȻ�
				
				encoder_integral_count();
				break;
			}
			
			case dif_in:    //ʹ����������
			{
				gyro_handle(0);   //�õ��Ƕ����
				//pos_control(0);    
				speed_control(2);  //�����õ�����ת��
				
				duty_get_close(0);
				break;
			}
			case auto_trail : //ʹ�õ��ѭ��,ע�������ǿ�������
			{
				gyro_handle(1);
				pos_control();    //ʹ�û��ڵķ���
				speed_control(4);    //�ٶȻ�pid�¸�һ�� λ�û�pid���� �ٶȸ�Ϊ�����ٶ�
				duty_get_close(2);   
				break;
			}
			case gyro_stright :   //ʹ��������ѭ��
			{
				gyro_handle(1);
				//pos_control(1);  
				gyro_out_pos=(roud_gy_d*(Angle_err-Last_Angle_err)+roud_gy_p*(Angle_err));  //����angle_err���ڴ���Ҫ��err������С
				speed_control(3); 
				duty_get_close(2);  
				break;
			}
			case out_round:  //��һ��ֱ��
			{
				pos_control();
				speed_control(1);
				duty_get_close(1);
				encoder_integral_count();
				break;
			}
			case round_done:     //�ж�Ϊ����״̬
			{
				round_state=pre_round;
		    is_roundabout_flag=0;    //�������ٴγ����ж�
				encoder_integral_count();
				
				if(ele_dir==0)  //����
		{
			if(ele_count==ele_num-1)
			ele_count=0;
			else
			ele_count++;   //�õ���Ԫ�����������

		}
		else
		{
			if(ele_count==0)
			ele_count=ele_num-1;
			else
			ele_count--;
		}
		
		
			  break;
			}
		}
	//ʹ������ѭ��
}
void roudabout_factor_judge(void)
{
	if(encoder_integral_in<integral_in*1000)   //���������Խ�ʡһ��ռ䣬��������һ�����ֱ���
	round_state=pre_round;
	else if(abs(Angle-init_Angle)<=at_trace_pro*gyro_roll)   //ʹ��������ѭ��
  round_state=dif_in;
	else if(abs(Angle-init_Angle)<=out_pro*gyro_roll)   //ʹ�õ��ѭ��
	round_state=auto_trail;
	else if(abs(Angle_err)>5)		//ʹ��������ѭ��
	round_state=gyro_stright;
	else if(encoder_integral_out<integral_out*1000)
	round_state=out_round; 
	else
	round_state=round_done;

}
void roudabout_control(void)
{
 	
	roudabout_factor_judge();	
	duty_get_roundabout();		
	//������ѭ��������ʹ����������ѭ����������ѭ���������õ��ѭ��������ֵ���ٿ������ĸ������Բ��
}
//roudabout over

void car_run(void)
{ 
	switch(factor_flag)
	{
		case roll:
		{
			pos_control();   //���
			speed_control(0);
			duty_get_close(0);
			break;
		}
		case stright:
		{
			pos_control();   //��ֵҪСһ��
			speed_control(1);
			duty_get_close(1);
			break;
		}	
		case roundabout:
		{
			roudabout_control();
			break;
		}
		case cross:
		{
			err_induct=((god_Sqrt(induct2_filter)-god_Sqrt(induct3_filter))*1000)/(induct2_filter+induct3_filter); //ʮ�����м��������ѭ��
			pos_control();   //���
			speed_control(1); //ֱ���ٶ�
			duty_get_close(0);	
			break;
		}
	}
}
void gyro_get(void)
{
			gyro_acc_filter();
		  Last_Angle=Angle;
			Last_Angle_err=Angle_err;  //����ֻ��Angle��last_Angle,����Angle_err�����¸�ֵ���������߼���
//			Angle=AngleGet();
			Angle=Angel_Get_Kal();
}

void protection(void)
{
	int32 stop_err=50;
	if(stop_err>induct1_filter)  //Ϊ������������������
	{
		if(stop_err>induct4_filter)
		{
			start_flag=0;
			clear_flag=1;
		}
		}
}
		



