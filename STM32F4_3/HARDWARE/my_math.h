#ifndef __MY_MATH_H
#define __MY_MATH_H
#include "system.h"
#include "IMU_hand.h"

#define MIN(a, b) 	(((a) < (b)) ? (a) : (b))
#define MAX(a, b) 	(((a) > (b)) ? (a) : (b))
#define M_PIf       3.14159265358979323846f


float invSqrt(float x);
float FL_ABS(float x);
float COS(float x);
float SIN(float y);
float my_abs(float f);
float atan2_approx(float y, float x);
float acos_approx(float x);

void _set_val(_F32xyz *_out_data,_F32xyz *_in_data);
void  set_value(_F32xyz *_in_data,float value);
int16_t  LimitProcess(int16_t thr_in,int16_t thr_min,int16_t thr_max);
		
float VariableParameter(float error);

uint16_t my_limit(uint16_t in,uint16_t min);


#endif

