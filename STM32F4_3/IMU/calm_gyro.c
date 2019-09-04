#include "calm_gyro.h"
#include "calm_acc.h"
#include "flash.h"
_GYRO_CAL _gyro_calm = {0};

/*陀螺仪校准
*/
#define gyro_cal_sum_value 1000
void gyro_calm(_F32xyz *gyro_in)
{
	if(_gyro_calm.star_flag == true)//开始校准
	{
//#if PRINTF
//		printf("%2f    %2f    %2f\r\n",_gyro_orig_f.x,_gyro_orig_f.y,_gyro_orig_f.z);
//#endif
		if(_gyro_calm.i < gyro_cal_sum_value)//求平均值
		{                       
			_gyro_calm.offset.x += gyro_in->x; 
			_gyro_calm.offset.y += gyro_in->y;
			_gyro_calm.offset.z += gyro_in->z;

			_gyro_calm.i++;
		}
		else
		{
			_gyro_calm.i = 0;

			_gyro_calm.offset_f.x = _gyro_calm.offset.x / gyro_cal_sum_value;    //得到三轴的零偏
			_gyro_calm.offset_f.y = _gyro_calm.offset.y / gyro_cal_sum_value;    //得到三轴的零偏
			_gyro_calm.offset_f.z = _gyro_calm.offset.z / gyro_cal_sum_value;    //得到三轴的零偏
			
			set_value(&_gyro_calm.offset,0.0f);//清空数据缓存,以免多次校准时数据累加
			//调试打印
//#if PRINTF
//			printf("_gyro_calm  %0.2f \t %0.2f \t %0.2f\r\n",_gyro_calm.offset_f.x,
//															 _gyro_calm.offset_f.y,
//															 _gyro_calm.offset_f.z);
//endif
			//将陀螺仪数据写入flash
			Flash_Write_gyro();
			_gyro_calm.star_flag = false; //清除标准位
			_gyro_calm.finish_flag = true;
		}
	}
}



