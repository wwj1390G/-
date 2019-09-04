#include "my_math.h"


//   快速求平方根倒数
float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
//  求float型数据绝对值
float FL_ABS(float x)
{
   if(x < 0)  return -x;
	 else return x; 
}
/*   采用三角函数的泰勒展开式 求近似值*/
float COS(float x)
{
	float result;
  result = 1 - x * x/2;
	return result; 
}

float SIN(float y)
{
	float result;
  result = y - y * y * y /6;
	return result; 
}


void _set_val(_F32xyz *_out_data,_F32xyz *_in_data)
{
    _out_data->x = _in_data->x;
    _out_data->y = _in_data->y;
    _out_data->z = _in_data->z;
}


void  set_value(_F32xyz *_in_data,float value)
{
    _in_data->x = value;
    _in_data->y = value;
    _in_data->z = value;
}


float atan2_approx(float y, float x)
{
    #define atanPolyCoef1  3.14551665884836e-07f
    #define atanPolyCoef2  0.99997356613987f
    #define atanPolyCoef3  0.14744007058297684f
    #define atanPolyCoef4  0.3099814292351353f
    #define atanPolyCoef5  0.05030176425872175f
    #define atanPolyCoef6  0.1471039133652469f
    #define atanPolyCoef7  0.6444640676891548f

    float res, absX, absY;
    absX = fabsf(x);
    absY = fabsf(y);
    res  = MAX(absX, absY);
    if (res) res = MIN(absX, absY) / res;
    else res = 0.0f;
    res = -((((atanPolyCoef5 * res - atanPolyCoef4) * res - atanPolyCoef3) * res - atanPolyCoef2) * res - atanPolyCoef1) / ((atanPolyCoef7 * res + atanPolyCoef6) * res + 1.0f);
    if (absY > absX) res = (M_PIf / 2.0f) - res;
    if (x < 0) res = M_PIf - res;
    if (y < 0) res = -res;
    return res;
}

float acos_approx(float x)
{
    float xa = fabsf(x);
    float result = sqrtf(1.0f - xa) * (1.5707288f + xa * (-0.2121144f + xa * (0.0742610f + (-0.0187293f * xa))));
    if (x < 0.0f)
        return M_PIf - result;
    else
        return result;
}


/*限幅处理*/
int16_t  LimitProcess(int16_t thr_in,int16_t thr_min,int16_t thr_max)
{
	if(thr_in<thr_min)	
		return thr_min;
	if(thr_in>thr_max)	
		return thr_max;
}



float my_abs(float f)
{
	if (f >= 0.0f)
	{
		return f;
	}
	return -f;
}
uint16_t my_limit(uint16_t in,uint16_t min)
{
	if(in < min){return 0;}	
	return in;
}


/***********************************************
  * @brief  可变增益自适应参数
  * @param  None
  * @retval None
************************************************/
float VariableParameter(float error)
{
	float  result = 0;
	if(error < 0)
	{
		error = -error;
	}
	if(error >0.8f)
	{
		error = 0.8f;
	}
	result = 1 - 1.28f*error ;
	if(result < 0)
	{
		result = 0;
	}

	return result;
}


