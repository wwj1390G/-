#include "calm_mag.h"
#include "flash.h"

_MAG_CAL _mag_calm ={0};

#define mag_cal_sum_value 20000

void mag_calm(_F32xyz *mag_in)
{
	static int sum = 0;
	
	if(_mag_calm.star_flag == true)//开始校准
	{
		if(sum < mag_cal_sum_value)//求极限值
		{
			if(_mag_calm.Max.x > mag_in->x) _mag_calm.Max.x = mag_in->x ;
			if(_mag_calm.Min.x < mag_in->x) _mag_calm.Min.x = mag_in->x ;
				
			if(_mag_calm.Max.y > mag_in->y) _mag_calm.Max.y = mag_in->y;
			if(_mag_calm.Min.y < mag_in->y) _mag_calm.Min.y = mag_in->y;
			
			if(_mag_calm.Max.z > mag_in->z) _mag_calm.Max.z = mag_in->z;
			if(_mag_calm.Min.z < mag_in->z) _mag_calm.Min.z = mag_in->z;
			sum++;
		}
		else
		{
			_mag_calm.x_gain = 1;
			_mag_calm.y_gain = (_mag_calm.Max.x - _mag_calm.Min.x) / (_mag_calm.Max.y - _mag_calm.Min.y);
			_mag_calm.z_gain = (_mag_calm.Max.x - _mag_calm.Min.x) / (_mag_calm.Max.z - _mag_calm.Min.z);
			
			_mag_calm.offset.x = (_mag_calm.Max.x + _mag_calm.Min.x)/2.0f;
			_mag_calm.offset.y = (_mag_calm.Max.y + _mag_calm.Min.y)/2.0f;
			_mag_calm.offset.z = (_mag_calm.Max.z + _mag_calm.Min.z)/2.0f;

			Flash_Write_mag();
			sum = 0;
			_mag_calm.star_flag = false;//清校准标志
			_mag_calm.finish_flag = true;
		}
	}
}


