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

//需要调整的
//正转总是轻易达到限幅，导致轨迹不稳定
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


//平方根
float god_Sqrt(float x)
{
	
    float xhalf = 0.5f * x;
    int32 i = *(int32*)&x;
    i = 0x5f3759df - (i >> 1);
    x = *(float*)&i;
    x = x * (1.5f - xhalf * x * x);
	//精度在此应该够用了
    //x = x * (1.5f - xhalf * x * x);
    //x = x * (1.5f - xhalf * x * x);

    return 1 / x;
}
void err_get(void)
{
	last_err_induct=err_induct;
	//err_induct>0时，小车向右偏
	if((induct2_filter+induct3_filter)!=0)
	{

		//err_induct=((induct1_filter-induct4_filter)*3000)/pow((induct2_filter+induct3_filter),1.8);
		//根号差比合，用于两横两斜，更线性,mid600,brim270
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
	if(induct2_filter>=mid_int&&induct3_filter>=mid_int&&induct1_filter>=brim_int&&induct4_filter>=brim_int)  //给差速看err_induct正负
	{
		roudabout_flag=1;    //将环岛元素置1
		if(is_roundabout_flag==0)
			is_roundabout_flag=1;
	}
	
	return roudabout_flag;
}  //是环岛return 1
void factor_judgement(void)
{
	static factor_judge_enum last_flag;
	last_flag=factor_flag;

	if(ele[ele_count+2]!=0&&(roudabout_judge()||is_roundabout_flag==1))  //当环岛判定为2状态时，为出环，此时进入直道弯道判断
	{
		factor_flag=roundabout;    //环岛的判定晚点补充
		if(last_flag!=roundabout)   //第一次判断为环岛
		{
			init_Angle=Angle;       //记录当前陀螺仪的值
		if(ele[ele_count+2]==1)   //此情况为左转
		Finish_Angle=Angle+gyro_roll;  //结束圆环时的角度
		else   //右转时减少
	  Finish_Angle=Angle-gyro_roll;
		Angle_err=0;    //把Angel_err置零
		

			
		}

	}
	else if(factor_flag==cross||(induct5_filter>=cross_idt&&induct6_filter>=cross_idt))  //暂时就这个条件，竖电感暂不用来作前瞻预测工作
	{
		factor_flag=cross;
		if(err_induct>-str_roll_judge||err_induct<str_roll_judge)  //判断为十字后，四电感误差为直道时再恢复正常循迹
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
		//右转角度减小
		pos_pid_out=final_p*err_induct+final_d*(err_induct-last_err_induct);	
	}
	else
		pos_pid_out=int_p*err_induct+int_d*(err_induct-last_err_induct);

	

}

void speed_control(uint8 flag)
{
	  float set_spd_l=0;
	  float set_spd_r=0;
  //直道弯道并无所谓，因为直道弯道在位置环上已经做了区分
		last_err_spd_r=err_spd_r;
		last_err_spd_l=err_spd_l;
	
		switch(flag)    //直弯分离
		{
			case 0:   //弯道
			{
				set_spd_r=roll_spd+pos_pid_out;  //在设定速度上的偏差
				set_spd_l=roll_spd-pos_pid_out;    //按照比例得到转速较慢的轮子的期望速度
				err_spd_l=set_spd_l-spd_l;   //正
				err_spd_r=set_spd_r-spd_r;   //负
				break;
			}
			case 1:  //直道
			{
			if(factor_flag!=roundabout)
			{
				set_spd_l=str_spd-pos_pid_out;
				set_spd_r=str_spd+pos_pid_out;
			}
			else   //使得不受弯道速度影响
			{
				if(round_state!=out_round)    //出环
				{set_spd_l=roud_spd;   //环旁边就不用电感来规范轨迹了
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
			case 2:       //自主入环
			{
			if((int32)ele[ele_count+2]==1)   //左入环
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
			case 3:    //自主出环
			{
				set_spd_l=roud_spd+gyro_out_pos;
				set_spd_r=roud_spd-gyro_out_pos;
				err_spd_l=set_spd_l-spd_l;
				err_spd_r=set_spd_r-spd_r;
				break;
			}
			case 4:		//自主循迹
			{
			if(pos_pid_out>0)   //左转
			{
				set_spd_r=roud_spd+pos_pid_out;  //在设定速度上的偏差
				set_spd_l=roud_spd-pos_pid_out;    //按照比例得到转速较慢的轮子的期望速度
				err_spd_l=set_spd_l-spd_l;   //正
				err_spd_r=set_spd_r-spd_r;   //负
			}
			else //右转
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
	//pd控制前馈大小，界限不使用电感的直弯判定，使用一个新的判定方法
	//此处不确定是否要使用两套pid，看看实际效果
	//不知道是否要在一定误差区间内不做影响
	//左右轮是否要设置两套pid

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

	else   //环岛内判定
	{
		duty_left=roud_at_p*(err_spd_l-last_err_spd_l)+roud_at_i*(err_spd_l)+last_spd_out_l;
		duty_right=roud_at_p*(err_spd_r-last_err_spd_r)+roud_at_i*(err_spd_r)+last_spd_out_r;
	}


	
	
	//陀螺仪的使用
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
		//angle err有问题
		//Last_Angle_err=Angle_err;  //由于只用Angle和last_Angle,并且Angle_err会重新赋值，所以两者兼容
		if(flag==0)
		{
			Angle_err=Angle-(init_Angle+at_trace_pro*(Finish_Angle-init_Angle));   //正右负左,以0.5来提供误差
		}
		else
		{
			Angle_err=Angle-Finish_Angle;        //出环时要求的姿态是最后的角度   
		}
		//为了避免给另一处正常使用
		
}

void encoder_integral_count(void)
{
	if(round_state==pre_round)
	{
		if((int32)ele[ele_count+2]==1)
		encoder_integral_in+=spd_l;     //直接通过左计数器继续计数
	  else
		encoder_integral_in+=spd_r;
	}
	else if(round_state==out_round)
	{
		if((int32)ele[ele_count+2]==1)
		encoder_integral_out+=spd_l;     //直接通过左计数器继续计数
	  else
		encoder_integral_out+=spd_r;	
	}
//  else
//	encoder_integral=0;        //不用计数时就清零
	else if(round_state==round_done)
	{
		encoder_integral_in=0;
		encoder_integral_out=0;
	}
	
}
void duty_get_roundabout(void)
{
	//开始使用陀螺仪循迹	
		switch(round_state)
		{
			case pre_round:    //默认直走 
			{
				pos_control();
				speed_control(1);
				
				duty_get_close(0);   //提前使用环内速度环
				
				encoder_integral_count();
				break;
			}
			
			case dif_in:    //使用自主差速
			{
				gyro_handle(0);   //得到角度误差
				//pos_control(0);    
				speed_control(2);  //用来得到期望转速
				
				duty_get_close(0);
				break;
			}
			case auto_trail : //使用电感循迹,注意陀螺仪开关问题
			{
				gyro_handle(1);
				pos_control();    //使用环内的方向环
				speed_control(4);    //速度环pid新给一套 位置环pid不动 速度改为环岛速度
				duty_get_close(2);   
				break;
			}
			case gyro_stright :   //使用陀螺仪循迹
			{
				gyro_handle(1);
				//pos_control(1);  
				gyro_out_pos=(roud_gy_d*(Angle_err-Last_Angle_err)+roud_gy_p*(Angle_err));  //由于angle_err过于大，需要把err进行缩小
				speed_control(3); 
				duty_get_close(2);  
				break;
			}
			case out_round:  //走一段直道
			{
				pos_control();
				speed_control(1);
				duty_get_close(1);
				encoder_integral_count();
				break;
			}
			case round_done:     //判定为出环状态
			{
				round_state=pre_round;
		    is_roundabout_flag=0;    //用来作再次出环判定
				encoder_integral_count();
				
				if(ele_dir==0)  //正跑
		{
			if(ele_count==ele_num-1)
			ele_count=0;
			else
			ele_count++;   //得到的元素往上面递增

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
	//使用自主循迹
}
void roudabout_factor_judge(void)
{
	if(encoder_integral_in<integral_in*1000)   //这样做可以节省一点空间，不用再设一个积分变量
	round_state=pre_round;
	else if(abs(Angle-init_Angle)<=at_trace_pro*gyro_roll)   //使用陀螺仪循迹
  round_state=dif_in;
	else if(abs(Angle-init_Angle)<=out_pro*gyro_roll)   //使用电感循迹
	round_state=auto_trail;
	else if(abs(Angle_err)>5)		//使用陀螺仪循迹
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
	//当自主循迹结束后使用陀螺仪来循迹，陀螺仪循迹结束后用电感循迹，误差的值减少看是入哪个方向的圆环
}
//roudabout over

void car_run(void)
{ 
	switch(factor_flag)
	{
		case roll:
		{
			pos_control();   //弯道
			speed_control(0);
			duty_get_close(0);
			break;
		}
		case stright:
		{
			pos_control();   //给值要小一点
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
			err_induct=((god_Sqrt(induct2_filter)-god_Sqrt(induct3_filter))*1000)/(induct2_filter+induct3_filter); //十字用中间两个电感循迹
			pos_control();   //弯道
			speed_control(1); //直道速度
			duty_get_close(0);	
			break;
		}
	}
}
void gyro_get(void)
{
			gyro_acc_filter();
		  Last_Angle=Angle;
			Last_Angle_err=Angle_err;  //由于只用Angle和last_Angle,并且Angle_err会重新赋值，所以两者兼容
//			Angle=AngleGet();
			Angle=Angel_Get_Kal();
}

void protection(void)
{
	int32 stop_err=50;
	if(stop_err>induct1_filter)  //为了消除警告缩短条件
	{
		if(stop_err>induct4_filter)
		{
			start_flag=0;
			clear_flag=1;
		}
		}
}
		



