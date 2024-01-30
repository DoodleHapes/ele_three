#include "SEEKFREE_OLED.h"
#include "key.h"
#include "zf_delay.h"
#include "flash.h"
#include "oled_menu.h"
#include "zf_adc.h"
#include "pid_control.h"
#include "encoder.h"
#include "gyro.h"
#include "fuzzy_control.h"





//电压
float volt;


//菜单有关
uint8 start_flag=0;
uint8 clear_flag=0;
uint8 page=init;
uint8 flag_in=0;
uint8 sign_count=0;   //用于指示多出来的
uint8 row_count=0;
uint8 last_row_count=0;
uint8 max_page_pid=13;     //用来计算第二页中最多的数值
uint8 max_page_motion=1;
uint8 max_page_roundabout=16;
uint8 max_ele_num=4;



//滚动菜单
//可以长按来向下滚动菜单，但是滑到底将失效
//皮皮学长的表现方式可以通过修改库函数得到
//出入平安
void page_init(void)
{
	//在此页面发车
	oled_p6x8str(0,0,"volt:");
	oled_printf_float(42,0,volt,4,2);   //电池电压检测
 //八路电感
  
	
	
	oled_printf_int32(0,1,(int32)induct1_filter,4);
	oled_printf_int32(31,1,(int32)induct2_filter,4);
	oled_printf_int32(63,1,(int32)induct3_filter,4);
	oled_printf_int32(95,1,(int32)induct4_filter,4);
	oled_printf_int32(31,2,(int32)induct5_filter,4);
	oled_printf_int32(63,2,(int32)induct6_filter,4);
//	oled_printf_int32(63,2,(int32)induct7_filter,4);
//	oled_printf_int32(95,2,(int32)induct8_filter,4);
	oled_p6x8str(0,3,"eco_l");
	oled_p6x8str(0,4,"eco_r");
	oled_p6x8str(64,3,"f_p");
	oled_p6x8str(64,4,"f_d");
	oled_printf_float(92,3,final_p,3,3);
	oled_printf_float(92,4,final_d,3,1);
//	oled_printf_float(92,3,actual_err,3,1);
//	oled_printf_float(92,4,actual_d_err,3,1);		
	oled_p6x8str(0,5,"Angle_err:");
	oled_p6x8str(0,6,"Angle:");
	oled_printf_int32(30,3,(int32)spd_l,6);
	oled_printf_int32(30,4,(int32)spd_r,6);
	oled_printf_float(60,5,Angle_err,3,3);
  oled_printf_float(60,6,Angle,3,3);


	
	
	
	oled_p6x8str(45,7,"err:");
	oled_printf_float(70,7,err_induct,3,2);

	
}
void page_sort(void)
{
	oled_p6x8str(0,0,"PID");
	oled_p6x8str(0,1,"roundabout_page");
	oled_p6x8str(0,2,"pwm_factor");
	oled_p6x8str(0,3,"motion");
	oled_p6x8str(0,4,"ele_list");
	oled_p6x8str(0,5,"kalman");	
	oled_printf_int32(64,0,row_count,3);		


}
void page_pid(void)
{
	uint8 location;
	oled_printf_int32(110,0,sign_count,2);
	//if(row_count>7)
		//sign_count=row_count%7;
	//if(max_page_1-sign_count<=7)
		//sign_count=max_page_1-7;     //为了滑到下面去也不会继续往下显示
	if(0-sign_count>=0)
	{
		location=0-sign_count;
		oled_p6x8str(0,location,"p_zero:");
		oled_printf_float(75,(uint16)location,p_zero,3,1);
	}
	if(1-sign_count>=0)
	{
		location=1-sign_count;
		oled_p6x8str(0,location,"d_zero:");
		oled_printf_float(75,(uint16)location,d_zero,3,1);
	}
	if(2-sign_count>=0)
	{
		location=2-sign_count;
		oled_p6x8str(0,location,"dlt_p_sm:");
		oled_printf_float(75,(uint16)location,delta_p_sm,3,1);
	}
	if(3-sign_count>=0)
	{
		location=3-sign_count;
		oled_p6x8str(0,location,"dlt_p_bst:");
		oled_printf_float(75,(uint16)location,delta_p_bst,3,1);
	}
		if(4-sign_count>=0)
	{
		location=4-sign_count;
		oled_p6x8str(0,location,"dlt_d_sm:");
		oled_printf_float(75,(uint16)location,delta_d_sm,3,1);
	}
		if(5-sign_count>=0)
	{
		location=5-sign_count;
		oled_p6x8str(0,location,"dlt_d_bst:");
		oled_printf_float(75,(uint16)location,delta_d_bst,3,1);
	}
	if(6-sign_count>=0)
	{
		location=6-sign_count;
		oled_p6x8str(0,location,"spd_kp:");
		oled_printf_float(75,(uint16)location,spd_kp,3,2);
	}
	if(7-sign_count>=0)
	{
		location=7-sign_count;
		oled_p6x8str(0,location,"spd_ki:");
		oled_printf_float(75,(uint16)location,spd_ki,2,2);
	}
	if(8-sign_count>=0&&8-sign_count<=7)
	{
		location=8-sign_count;
		oled_p6x8str(0,location,"str_spd");
		oled_printf_int32(75,(uint16)location,str_spd,3);
	}

	if(9-sign_count>=0&&9-sign_count<=7)
	{
		location=9-sign_count;
		oled_p6x8str(0,location,"roll_spd");
		oled_printf_int32(75,(uint16)location,roll_spd,3);
	}
	if(10-sign_count>=0&&10-sign_count<=7)
	{
		location=10-sign_count;
		oled_p6x8str(0,location,"max_err");
		oled_printf_float(75,(uint16)location,max_err_bound,3,1);
	}
	if(11-sign_count>=0&&11-sign_count<=7)
	{
		location=11-sign_count;
		oled_p6x8str(0,location,"max_d_err");
		oled_printf_float(75,(uint16)location,max_d_err_bound,3,1);
	}
	if(12-sign_count>=0&&12-sign_count<=7)
	{
		location=12-sign_count;
		oled_p6x8str(0,location,"cross_idt");
		oled_printf_int32(75,(uint16)location,cross_idt,3);
	}
}
void page_factor(void)
{
	oled_p6x8str(12,0,"s_r_judge:");
		oled_printf_float(75,0,str_roll_judge,4,1);
	oled_p6x8str(0,1,"dt_l:");
	if(duty_left>0)
	oled_printf_int32(35,1,duty_left,4);
	else
	{
		oled_p6x8str(35,1,"    ");
		oled_printf_int32(35,1,0,4);

	}
	oled_p6x8str(60,1,"dt_r:");
	if(duty_right>0)
	oled_printf_int32(92,1,duty_right,4);
	else
	{
		oled_p6x8str(92,1,"    ");
		oled_printf_int32(92,1,0,4);
		

	}
	oled_printf_int32(35,2,convert_l,4);
	oled_printf_int32(92,2,convert_r,4);
	oled_p6x8str(0,2,"cv_l:");
	oled_p6x8str(60,2,"cv_r:");

	
	
	
	//用于判断直弯
	if(factor_flag==roll)  
	oled_p6x8str(64,4,"roll");
	else if(factor_flag==stright)
	{
		oled_p6x8str(64,4,"    ");
		oled_p6x8str(64,4,"str");
	}
	else if(factor_flag==roundabout)
	{
		oled_p6x8str(64,4,"    ");
		oled_p6x8str(64,4,"round");
	}
	else if(factor_flag==cross)
	{
		oled_p6x8str(64,4,"    ");
		oled_p6x8str(64,4,"cross");	
	}
		
}
void page_motion(void)
{
	uint8 i;
	oled_p6x8str(0,0,"gyro_motion");  //陀螺仪模式
 for(i=0;i<max_page_motion;i++)
 {

		 if(((on_off_motion[i])&1)==1)//奇数
		oled_p6x8str(92,i,"OFF");
	else
		oled_p6x8str(92,i,"ON ");
 }
}
	

void page_roundabout(void)
{
	uint8 location;
	if(0-sign_count>=0)
	{
		location=0-sign_count;
		oled_p6x8str(0,location,"gyro_roll:");
		oled_printf_int32(92,(uint16)location,(int32)gyro_roll,3);
	}
	if(1-sign_count>=0)
	{
		location=1-sign_count;
		oled_p6x8str(0,location,"at_trace_pro:");
		oled_printf_float(92,(uint16)location,at_trace_pro,1,2);
	}
	if(2-sign_count>=0)
	{
		location=2-sign_count;
		oled_p6x8str(0,location,"out_pro:");
		oled_printf_float(92,(uint16)location,out_pro,1,2);
	}
	if(3-sign_count>=0)
	{
		location=3-sign_count;
		oled_p6x8str(0,location,"intg_in:");
		oled_printf_int32(92,(uint16)location,(int32)integral_in,3);
	}
	if(4-sign_count>=0)
	{
		location=4-sign_count;
		oled_p6x8str(0,location,"intg_out:");
		oled_printf_int32(92,(uint16)location,(int32)integral_out,3);
	}
	if(5-sign_count>=0)
	{
		location=5-sign_count;
		oled_p6x8str(0,location,"rd_in_dif:");
		oled_printf_float(92,(uint16)location,roud_in_dif,3,1);
	}
	if(6-sign_count>=0&&6-sign_count<=7)
	{
		location=6-sign_count;
		oled_p6x8str(0,location,"int_p");
		oled_printf_int32(92,(uint16)location,int_p,4);
	}
	if(7-sign_count>=0&&7-sign_count<=7)
	{
		location=7-sign_count;
		oled_p6x8str(0,location,"int_d");
		oled_printf_int32(92,(uint16)location,int_d,4);
	}

	if(8-sign_count>=0&&8-sign_count<=7)
	{
		location=8-sign_count;
		oled_p6x8str(0,location,"roud_at_p:");
		oled_printf_float(92,(uint16)location,roud_at_p,2,2);

	}
	if(9-sign_count>=0&&9-sign_count<=7)
	{
		location=9-sign_count;
		oled_p6x8str(0,location,"roud_at_i:");
		oled_printf_float(92,(uint16)location,roud_at_i,2,2);
	}
	if(10-sign_count>=0&&10-sign_count<=7)
	{
		location=10-sign_count;
		oled_p6x8str(0,location,"roud_gy_p");
		oled_printf_float(92,(uint16)location,(int32)roud_gy_p,4,1);
	}
	if(11-sign_count>=0&&11-sign_count<=7)
	{
		location=11-sign_count;
		oled_p6x8str(0,location,"roud_gy_d");
		oled_printf_float(92,(uint16)location,(int32)roud_gy_d,4,1);
	}
	if(12-sign_count>=0&&12-sign_count<=7)
	{
		location=12-sign_count;
		oled_p6x8str(0,location,"roud_spd");
		oled_printf_int32(92,(uint16)location,(int32)roud_spd,4);
	}
	if(13-sign_count>=0&&13-sign_count<=7)
	{
		location=13-sign_count;
		oled_p6x8str(0,location,"brim_int");
		oled_printf_int32(92,(uint16)location,(int32)brim_int,4);
	}	
	if(14-sign_count>=0&&14-sign_count<=7)
	{
		location=14-sign_count;
		oled_p6x8str(0,location,"mid_int");
		oled_printf_int32(92,(uint16)location,(int32)mid_int,4);
	}	
	
  oled_printf_int32(70,0,(int32)round_state,1);   //显示在哪个状态
}
void page_kalman(void)
{
	oled_p6x8str(0,0,"klm_R:");
 	oled_p6x8str(0,1,"klm_Q:"); 
	oled_p6x8str(0,2,"klm_B:");
	oled_p6x8str(0,3,"klm_LastP:");
	oled_printf_float(92,0,klm_R,3,2);	
	oled_printf_float(92,1,klm_Q,3,2);
	oled_printf_float(92,2,klm_B,3,2);
}
void element_judge(uint8 i)
{
	switch(ele[i])
	{
		case 0:
		{
			oled_p6x8str(80,i,"Off");
			break;
		}
		case 1:
		{
			oled_p6x8str(80,i,"Lef_Rd");
			break;
		}
		case 2:
		{
			oled_p6x8str(80,i,"Rig_Rd");
			break;
		}
		case 3:
		{
			oled_p6x8str(80,i,"B_Lef_Rd");	
			break;
		}
		case 4:
		{
			oled_p6x8str(80,i,"B_Rig_Rd");
			break;
		}
		default:
		{
			break;
		}
		
	}
}
void page_ele_list(void)
{
	uint8 i;
	oled_p6x8str(0,0,"direction:");
	if(ele_dir==0)
	oled_p6x8str(80,0,"pos");
	else
	oled_p6x8str(80,0,"cov");
	
	oled_p6x8str(0,1,"num:");
	oled_printf_int32(80,1,(int32)ele_num,2);
	
	oled_p6x8str(0,2,"Fir:");
	oled_p6x8str(0,3,"Sec:");
	oled_p6x8str(0,4,"Thr:");
	oled_p6x8str(0,5,"For:");
	oled_p6x8str(0,6,"Fiv:");
	
	for(i=2;i<=6;i++)
	element_judge(i);
	
	
}
	
void oled_show_flag(void)
{
	
	//oled_printf_int32(50,1,row_count,4);
	if(row_count<7)   //当小于7的时候就在
	{	
		if(flag_in==1)
		{
			
			oled_p6x8str(120,row_count,"<");
			oled_p6x8str(70,row_count,"*");
		}
		else
		{
			
			oled_p6x8str(120,row_count,"<");
			
		}
	}
	else              //当滑到底时停留在最后一行
	{
		if(flag_in==1)
		{
			
			oled_p6x8str(120,7,"<");
			oled_p6x8str(70,7,"*");
		}
		else
		{
			
			oled_p6x8str(120,7,"<");			
			
		}		
	}
}
void oled_show_image(void)
{
	if(clear_flag==1)
	{
		clear_flag=0;
		oled_clear();
	}
	switch(page)
	{
		case init:          //init
		{
			page_init();
			break;
		}
		case sort:
		{
			page_sort();
			break;
		}
		case PID:         //PID
		{
			page_pid();
			break;
		}
		case pwm_factor:         //pwm_factor
		{
			page_factor();
			break;
		}
		case roundabout_page:        //roundabout_page
		{
			page_roundabout();
			break;
		}
		case motion:				//motion
		{
			page_motion();
			break;
		}
		case ele_list:       //
		{
			page_ele_list();
			break;
		}
		case kalman:
		{
			page_kalman();
			break;
		}
	}
	if(page!=init)
	oled_show_flag();
	
}
void change_num(uint8 i)   //写出对应的值
{
	if(i==0)    //cut
	{
		switch(page)
		{
			case PID:
			{
					switch(row_count)
					{
						case 0:
						{
							p_zero--;
							break;
						}
						case 1:
						{
							d_zero--;
							break;
						}
						case 2:
						{
							delta_p_sm--;
							break;
						}
						case 3:
						{
							delta_p_bst--;
							break;
						}
						case 4:
						{
							delta_d_sm--;
							break;
						}						
						case 5:
						{
							delta_d_bst--;
							break;
						}

						case 6:
						{
							spd_kp--;
							break;
						}
						case 7:
						{
							spd_ki--;
							break;
						}
					case 8:
					{
						str_spd--;
						break;
					}
					case 9:
					{
						roll_spd--;
						break;
					}
					case 10:
					{
						max_err_bound-=1;
						break;
					}
					case 11:
					{
						max_d_err_bound-=1;							
						break;
					}
					case 12:
					{
						cross_idt-=50;
						break;
					}
						default:
						{
							break;
						}
					
					}
					break;
			}
			case roundabout_page:
		{
			switch(row_count)
			{
				case 0:
				{
					gyro_roll--;
					break;
				}
				case 1:
				{
					at_trace_pro-=0.01;
					break;
				}
				case 2:
				{
					out_pro-=0.01;
					break;
				}			
				case 3:
				{
					integral_in-=1;
					break;
				}			
				case 4:
				{
					integral_out-=1;
					break;
				}				
				case 5:
				{
					roud_in_dif-=1;
					break;
				}	
				case 6:
				{
					int_p-=1;
					break;
				}
				case 7:
				{
					int_d-=1;
					break;
				}					
				case 8:
				{
					roud_at_p-=1;
					break;
				}
				case 9:
				{
					roud_at_i-=1;
					break;
				}		
				case 10:
				{
					roud_gy_p-=1;
					break;
				}
				case 11:
				{
					roud_gy_d-=1;
					break;
				}
				case 12:
				{
					roud_spd-=1;
					break;
				}
				case 13:
				{
					brim_int-=10;
					break;
				}
				case 14:
				{
					mid_int-=10;
					break;
				}
				
				
			}
		break;
		}
			case pwm_factor:   //当在第三页
			{
				str_roll_judge-=0.1;
				break;
			}
			case ele_list:
			{
				if(row_count!=0)   //调整个数
				{
					if(ele[row_count]>0)
					ele[row_count]--;
				}				
				break;
			}
				
			
			
			case kalman:
			{
				switch(row_count)
				{
					case 0:
					{
						klm_R-=0.1;
						break;
					}
					case 1:
					{
						klm_Q-=0.01;
						break;
					}	
					case 2:
					{
						klm_B-=0.01;
						break;
					}		
				}
				break;
			}
			default :
			{
				break;
			}
			break;
		}
		
		
	}

	else        //add
	{
			switch(page)
			{
				case PID:
		{
			switch(row_count)
			{
					case 0:
						{
							p_zero++;
							break;
						}
						case 1:
						{
							d_zero++;
							break;
						}
						case 2:
						{
							delta_p_sm++;
							break;
						}
						case 3:
						{
							delta_p_bst++;
							break;
						}
						case 4:
						{
							delta_d_sm++;
							break;
						}						
						case 5:
						{
							delta_d_bst++;
							break;
						}

						case 6:
						{
							spd_kp++;
							break;
						}
						case 7:
						{
							spd_ki++;
							break;
						}
					case 8:
					{
						str_spd++;
						break;
					}
					case 9:
					{
						roll_spd++;
						break;
					}
					case 10:
					{
						max_err_bound+=1;
						break;
					}
					case 11:
					{
						max_d_err_bound+=1;							
						break;
					}
					case 13:
					{
						cross_idt+=50;
						break;
					}
					default:
					{
						break;
					}
					}
			
				break;
			
		}
			case roundabout_page:
		{
			switch(row_count)
			{
				case 0:
				{
					gyro_roll++;
					break;
				}
				case 1:
				{
					at_trace_pro+=0.01;
					break;
				}
				case 2:
				{
					out_pro+=0.01;
					break;
				}			
				case 3:
				{
					integral_in+=1;
					break;
				}			
				case 4:
				{
					integral_out+=1;
					break;
				}				
				case 5:
				{
					roud_in_dif+=1;
					break;
				}	
				case 6:
				{
					int_p+=1;
					break;
				}
				case 7:
				{
					int_d+=1;
					break;
				}					
				case 8:
				{
					roud_at_p+=1;
					break;
				}
				case 9:
				{
					roud_at_i+=1;
					break;
				}		
				case 10:
				{
					roud_gy_p+=1;
					break;
				}
				case 11:
				{
					roud_gy_d+=1;
					break;
				}
				case 12:
				{
					roud_spd+=1;
					break;
				}
				case 13:
				{
					brim_int+=10;
					break;
				}
				case 14:
				{
					mid_int+=10;
					break;
				}					
				
			}
		break;
		}
			case motion:
		{
			if(on_off_motion[row_count]==1)
			on_off_motion[row_count]=2;
			else
				on_off_motion[row_count]=1;
			break;
		}
			case pwm_factor:   //当在第三页
			{
				str_roll_judge+=0.1;
				break;
			}
			case ele_list:
			{
				if(row_count!=1)
				{
					if(row_count!=0)
				{
					if(ele[row_count]!=max_ele_num)
					ele[row_count]++;
					else
					ele[row_count]=0;
				}
				else
				{
					if(ele[row_count]==1)
						ele[row_count]=0;
					else
						ele[row_count]=1;
				}
				}
				else 
					ele[row_count]++;
				
				break;
			}
			case kalman:
			{
				switch(row_count)
				{
					case 0:
					{
						klm_R+=0.1;
						break;
					}
					case 1:
					{
						klm_Q+=0.01;
						break;
					}	
					case 2:
					{
						klm_B+=0.01;
						break;
					}			
				}

				break;
			}
			default :
			{
				break;
			}
	}
}
	}
void key_handle_short(uint8 key_num)
{
	uint8 max_page_sort=7;
	switch(key_num)
	{
		case 0:    //up
		{
			if(page!=init)
			{if(flag_in!=1)
			{
				
				if(row_count<=0)
					row_count=0;
				else
					row_count--;
				clear_flag=1;
				if(row_count>=7)
				{
					if(row_count<14)
					sign_count=row_count%7;
				else
					sign_count=7+row_count%7;
				}
			}
			}
			
			break;
		}
		case 1:    //down
		{
			if(page!=init)
			{
				if(flag_in!=1)
			{
				row_count++;

					if(row_count>7)
					{
						if(row_count<14)
					sign_count=row_count%7;
				else
					sign_count=7+row_count%7;
					}
				if(page==sort)
				{
					if(row_count>=max_page_sort)
						row_count--;
						
				}
				if(page==PID)    //第二页的情况
					{
						if(row_count>=max_page_pid)    //当滑到底
						
				{
					sign_count=max_page_pid-7;     //为了滑到下面去也不会继续往下显示
					row_count--;   //与前面的自增相抵消
				}
			}
				if(page==roundabout_page)    //第五页的情况
					{
						if(row_count>=max_page_roundabout)    //当滑到底
						
				{
					sign_count=max_page_roundabout-7;     //为了滑到下面去也不会继续往下显示
					row_count--;   //与前面的自增相抵消
				}
			}					
					clear_flag=1;
			
			}
			}
			break;
		}
		case 2:   //left
		{
			clear_flag=1;
			if(page!=init)  //当不是	
			{
				if(flag_in==0)
			{
				if(page!=sort)
				{
					sign_count=0;
					row_count=last_row_count;
					clear_flag=1;
					page=sort;     //到挑选进入sort环节
				}
				else
				{
					sign_count=0;
					row_count=0;
					last_row_count=0;
					clear_flag=1;
					page=init;     			
				}
				//通过独立按键2来回到初始页面
			}
			
			else
			{
				change_num(0);   //改变对应的值
			}
		}
			else
			{
				clear_flag=1;
				volt=adc_once(ADC_P14,ADC_10BIT)/37.3;    //得到采集的电感值
				
			}
				
			break;
		}
		case 3:   //right
		{
			clear_flag=1;

				if(page==init)
			{
				sign_count=0;
				row_count=0;
				clear_flag=1;
				page=sort;
			
			}
			else if(page==sort)
			{
				
				clear_flag=1;
				if(row_count==0)    //进入PID界面
					page=PID;
				else if(row_count==1)
					page=roundabout_page;
				else if(row_count==2)
					page=pwm_factor;
				else if(row_count==3)
					page=motion;
				else if(row_count==4)
					page=ele_list;
				else if(row_count==5)
					page=kalman;
				last_row_count=row_count;
				sign_count=0;
				row_count=0;
			
			}
			
			else if(flag_in==1)
			{
				change_num(1);
			}
			break;
		}
		case 4:   //mid
		{
			if(flag_in==1)
				flag_in=0;
			else flag_in=1;
			clear_flag=1;
			break;
		}
		case 5:   //独立按键1
		{
			if(page!=init)
			{
				flash_write();     //将数据写进str中
				ele_init();   //重新赋值一次
				oled_clear();
				oled_p6x8str(64,4,"flashed");
				delay_ms(500);
				oled_clear();
			}
			else
			{
				
				if((gyro_motion&1)==1)
				{
					oled_clear();
				oled_p6x8str(64,4,"gyro_started");
				//get_gyro_err();   //重新把误差去除
				gyro_motion++;   //打开陀螺仪
				delay_ms(100);
				oled_clear();
				}
				else
				{
				 oled_clear();
				 oled_p6x8str(64,4,"gyro_closed");
				 //get_gyro_err();   //重新把误差去除
					gyro_motion++;   //打开陀螺仪
				 delay_ms(100);
				 oled_clear();				
				}
			}
			
			
			break;
		}
		case 6:   //独立按键2
		{
			if(start_flag==0)
			{
				if(page!=init)
			{
				page=init;     //回到首页
				last_row_count=0;
				row_count=0;
				sign_count=0;
				clear_flag=1;
			}
				else
			{
				fuzzy_init();  //把fuzzy参数放到计算中去
				delay_ms(500);   //按下按键延迟发车
				clear_flag=1;
				start_flag=1;

			}
			}
			else
			{
				start_flag=0;
				clear_flag=1;
			}
				
			
			break;
		}
	}
}
void key_handle_long(uint8 key_num)
{
	if(key_num==6)    //按独立按键2时
	{
		flash_read();    //重新读取一次，回到原始数值
		oled_clear();
		oled_p6x8str(64,4,"reflashed");
		oled_clear();
		delay_ms(500);
	}
	else
	{
		delay_ms(10);
		key_handle_short(key_num);

	}
}
void key_handle(void)
{
	uint8 key_num;     //用来计算第几个按键
	
	for(key_num=0;key_num<key_number;key_num++)
	{
		switch(key_state[key_num])
	{
		case 0:     //release
		{
			break;
		}
		case 1:     //short    
		{
			key_handle_short(key_num);
			break;
		} 
		case 2:     //long
		{
			
			key_handle_long(key_num);    //每隔一段时间进行一次改变
			break;
		}
	}
}

}


void oled_menu(void)
{
	key_handle();
	if(start_flag==0)
	oled_show_image();
	else
	{
		page_init();  //显示几个电感值方便观察
		oled_p6x8str(64,4,"Running");
	}
}



