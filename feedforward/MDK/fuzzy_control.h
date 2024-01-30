#ifndef __FUZZY_CONTROL_H__
#define __FUZZY_CONTROL_H__

#include "common.h"

struct fuzzy_TypeDef
{
  uint8 delta_x;
	uint8 p[3];  //zero,smal,big
	uint8 d[5];  //nb,ns,z,ps,pb
	float	degree_mem_p[5];  //p的隶属度数组
	float	degree_mem_d[5];	//d的隶属度数组	
};


extern float final_p;  //最终的p
extern float final_d;	//最终的d
extern struct fuzzy_TypeDef fuzzy; 


void fuzzy_init(void);
void fuzzy_pid(float err,float d_err);
#endif