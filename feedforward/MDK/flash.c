#include "flash.h"
#include "common.h"
#include "zf_eeprom.h"
#include "math.h"
uint8 number=35;
uint8 write_buff[4*45];			//写入eeprom的数据
uint8 read_buff[4*45];		//读出eeprom的数据 
uint32 Flash_Index=0x00;
uint8 on_off_motion[5];
uint8 ele[6];
//PID参数
struct flash_number flash;
struct roudabout round;
struct Kalman_Typedef Klm;
uint8 lastP;
//规定好几个位置来放置，然后取每位数进行进制换算





float Read_Flash(void)
{
	float Num=0;
	uint8 Len=0;
	uint8 Read_Char[1],Minus_Flag=0;
	iap_read_bytes(Flash_Index,Read_Char,1);
	while(Read_Char[0]!='\0')
	{
		if(Read_Char[0]>='0'&&Read_Char[0]<='9')
		{
			Num+=Read_Char[0]-'0';
			Num/=10;
			Len++;
		}
		else if(Read_Char[0]=='.')
		{
			Len=0;
		}
		else if(Read_Char[0]=='-')
		{
			Minus_Flag=1;
		}
		Flash_Index++;
		iap_read_bytes(Flash_Index,Read_Char,1);
	}
	Flash_Index++;
	while(Len!=0)
	{
		Num*=10;
		Len--;
	}
	if(Minus_Flag==1)
		Num=0-Num;
	return Num;
}


void Write_Flash(float Num,uint8 Bit)
{
	uint8 Int_Len=0,Float_Len=0;
	uint8 Write_Char[12];
	int32 Data;
	if(Bit==0)
	{
		Data=(int32)(fabs(Num));
		while(Data!=0)
		{
			Write_Char[Int_Len]=Data%10+'0';
			Data/=10;
			Int_Len++;
		}
		if(Int_Len==0)
		{
			Write_Char[0]='0';
			Write_Char[1]='\0';
			Int_Len=2;
		}
		else
		{
			if(Num<0)
			{
				Write_Char[Int_Len]='-';
				Write_Char[Int_Len+1]='\0';
				Int_Len+=2;
			}
			else
			{
				Write_Char[Int_Len]='\0';
				Int_Len++;
			}
		}
		iap_write_bytes(Flash_Index,Write_Char,Int_Len);
		Flash_Index+=Int_Len;
	}
	else
	{
		for(Int_Len=0;Int_Len<Bit;Int_Len++)
		{
			Num*=10;
		}
		Int_Len=0;
		Data=(int32)(fabs(Num));
		while(Int_Len<Bit)
		{
			Write_Char[Int_Len]=Data%10+'0';
			Data/=10;
			Int_Len++;
		}
		Write_Char[Int_Len]='.';
		Int_Len++;
		while(Data!=0)
		{
			Write_Char[Int_Len+Float_Len]=Data%10+'0';
			Data/=10;
			Float_Len++;
		}
		if(Float_Len==0)
		{
			Write_Char[Int_Len]='0';
			if(Num<0)
			{
				Write_Char[Int_Len+1]='-';
				Write_Char[Int_Len+2]='\0';
				Float_Len+=3;
			}
			else
			{
				Write_Char[Int_Len+1]='\0';
				Float_Len+=2;
			}
		}
		else
		{
			if(Num<0)
			{
				Write_Char[Int_Len+Float_Len]='-';
				Write_Char[Int_Len+Float_Len+1]='\0';
				Float_Len+=2;
			}
			else
			{
				Write_Char[Int_Len+Float_Len]='\0';
				Float_Len++;
			}
		}
		iap_write_bytes(Flash_Index,Write_Char,Int_Len+Float_Len);
		Flash_Index=Flash_Index+Int_Len+Float_Len;
	}
}
void flash_read(void)
{
	Flash_Index=0;
	
	
	p_zero=Read_Flash();
	d_zero=Read_Flash();
	delta_p_sm=Read_Flash();
	delta_p_bst=Read_Flash();
	delta_d_sm=Read_Flash();
	delta_d_bst=Read_Flash();
	str_roll_judge=Read_Flash();
  max_err_bound=Read_Flash();	
	on_off_motion[0]=Read_Flash();
	on_off_motion[1]=Read_Flash();
	spd_kp=Read_Flash();
	spd_ki=Read_Flash();
	gyro_roll=Read_Flash();
	at_trace_pro=Read_Flash();
	out_pro=Read_Flash();
	roud_in_dif=Read_Flash();
	klm_R=Read_Flash();
	klm_Q=Read_Flash();
	klm_B=Read_Flash();	
	lastP=Read_Flash();
	str_spd=Read_Flash();
	roll_spd=Read_Flash();

	integral_in=Read_Flash();
	integral_out=Read_Flash();
	roud_at_p=Read_Flash();
	roud_at_i=Read_Flash();
	roud_gy_p=Read_Flash();
	roud_gy_d=Read_Flash();
	brim_int=Read_Flash();
	mid_int=Read_Flash();
	roud_spd=Read_Flash();
	int_p=Read_Flash();
	int_d=Read_Flash();
	ele_num=Read_Flash();
	ele_Fir=Read_Flash();
	ele_Sec=Read_Flash();
	ele_Thr=Read_Flash();
	ele_For=Read_Flash();
	ele_Fiv=Read_Flash();
	ele_dir=Read_Flash();
	max_d_err_bound=Read_Flash();
	cross_idt=Read_Flash();
}
void flash_write(void)
{
//	uint8 i;
	float Input=1;
	Flash_Index=0;
	iap_erase_page(0);    //刷新页区
	//iap_erase_page(520);
	

	Write_Flash(p_zero,3);
	Write_Flash(d_zero,3);
	Write_Flash(delta_p_sm,3);
	Write_Flash(delta_p_bst,4);
  Write_Flash(delta_d_sm,4);
	Write_Flash(delta_d_bst,3);
	Write_Flash(str_roll_judge,4);	
  Write_Flash(max_err_bound,3);	
	Write_Flash((float)on_off_motion[0],3);
	Write_Flash((float)on_off_motion[1],3);
	Write_Flash(spd_kp,3);
	Write_Flash(spd_ki,3);
	Write_Flash(gyro_roll,2);
	Write_Flash(at_trace_pro,2);
	Write_Flash(out_pro,2);
	Write_Flash(roud_in_dif,3);
	Write_Flash(klm_R,3);  //R
	Write_Flash(klm_Q,3);  //Q
	Write_Flash(klm_B,3);     //B
	Write_Flash(lastP,3);  //lastp
	Write_Flash(str_spd,3);
	Write_Flash(roll_spd,3);
	Write_Flash(integral_in,3);
	Write_Flash(integral_out,3);
	Write_Flash(roud_at_p,3);
	Write_Flash(roud_at_i,3);
	Write_Flash(roud_gy_p,3);
	Write_Flash(roud_gy_d,3);
	Write_Flash(brim_int,3);
	Write_Flash(mid_int,3);
	Write_Flash(roud_spd,3);
	Write_Flash(int_p,3);
	Write_Flash(int_d,3);
	Write_Flash(ele_num,3);
	Write_Flash(ele_Fir,3);
	Write_Flash(ele_Sec,3);
	Write_Flash(ele_Thr,3);
	Write_Flash(ele_For,3);
	Write_Flash(ele_Fiv,3);
	Write_Flash(ele_dir,3);
	Write_Flash(max_d_err_bound,3);
	Write_Flash(cross_idt,3);

}
void flash_number_init(void)
{
	p_zero=5;   
	d_zero=25; 
	delta_p_sm=3;
	delta_p_bst=9;
	delta_d_sm=10;
	delta_d_bst=15;
	spd_kp=80;
	spd_ki=26;
	str_spd=275;
	roll_spd=260;
	max_err_bound=40;
	max_d_err_bound=3;
	cross_idt=500;
 //环岛参数
 gyro_roll=355;
 at_trace_pro=0.20;
 out_pro=0.78;
 roud_in_dif=200;
 integral_in=4;
 integral_out=4;
	roud_at_p=40;
	roud_at_i=20;
	roud_gy_p=3;
	roud_gy_d=10;
	brim_int=700;
	mid_int=900;
	roud_spd=255;
 int_p=12;
 int_d=48;

	
//其余参数
 str_roll_judge=2;
 gyro_motion=2;
 idt_t_motion=2;
 feed_forward=2;
 gy_rd_motion=2;
 gy_bl_motion=1;
	

	//元素表
	ele_num=2;
	ele_dir=0;
	ele_Fir=0;
	ele_Sec=0;
	ele_Thr=0;
	ele_For=0;
	ele_Fiv=0;
  //卡尔曼
	klm_R=1;
	klm_Q=0.01;
	klm_B=0;
	lastP=0;
	
	
}
