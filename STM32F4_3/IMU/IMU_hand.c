#include "IMU_hand.h"
#include "IMU.h"
#include "filter.h"
#include "calm_acc.h"
#include "calm_gyro.h"
#include "calm_mag.h"

_MPU _Mpu = {0};//陀螺仪 原始数据
_PRESSURE _MS5611 = {0};//气压计ms5611的值

//原始加速度量-->转化为 g
void get_acc_g(_F32xyz *acc_in,_F32xyz *acc_out)
{
		acc_out->x = (float)(acc_in->x * acc_raw_to_g);
		acc_out->y = (float)(acc_in->y * acc_raw_to_g);
		acc_out->z = (float)(acc_in->z * acc_raw_to_g);
}

//raw -> rad/s	转/秒
void get_rad_s(_F32xyz *gyro_in,_F32xyz *gyro_out)
{
	gyro_out->x = (float)(gyro_in->x * gyro_raw_to_radian_s);
	gyro_out->y = (float)(gyro_in->y * gyro_raw_to_radian_s);
	gyro_out->z = (float)(gyro_in->z * gyro_raw_to_radian_s);
}

//raw -> deg/s  度/秒
void get_deg_s(_F32xyz *gyro_in,_F32xyz *gyro_deg_out)
{
	gyro_deg_out->x = (float)(gyro_in->x * gyro_raw_to_deg_s);
	gyro_deg_out->y = (float)(gyro_in->y * gyro_raw_to_deg_s);
	gyro_deg_out->z = (float)(gyro_in->z * gyro_raw_to_deg_s);    
}

_Butterworth_data   gyro_butter_data[3];
//对陀螺仪原始数据进行偏零校准
void gyro_calibration(_F32xyz* gyro_f)
{
	mpu6000GyroRead(&_Mpu.gyro);//得到陀螺仪值(原始值)
	
//	//偏零校准
//	_Mpu.gyro.x = _Mpu.gyro.x - _Mpu.gyro_offset.x;
//	_Mpu.gyro.y = _Mpu.gyro.y - _Mpu.gyro_offset.y;
//	_Mpu.gyro.z = _Mpu.gyro.z - _Mpu.gyro_offset.z;
	_Mpu.gyro.x = _Mpu.gyro.x - _gyro_calm.offset_f.x;
	_Mpu.gyro.y = _Mpu.gyro.y - _gyro_calm.offset_f.y;
	_Mpu.gyro.z = _Mpu.gyro.z - _gyro_calm.offset_f.z;
	

	//butterworth滤波
	gyro_f->x = (float)butterworth_lpf(((float)_Mpu.gyro.x),&gyro_butter_data[0],&gyro_30hz_parameter);
	gyro_f->y = (float)butterworth_lpf(((float)_Mpu.gyro.y),&gyro_butter_data[1],&gyro_30hz_parameter);
	gyro_f->z = (float)butterworth_lpf(((float)_Mpu.gyro.z),&gyro_butter_data[2],&gyro_30hz_parameter);
}



//加速度计校准
void acc_calibration(_F32xyz* acc_f)
{
	mpu6000AccRead(&_Mpu.acc);//得到加速度计(原始值)
	
	//校准	
	acc_f->x = (float)(_acc_calm.K[0]*((float)_Mpu.acc.x) - _acc_calm.B[0]*one_g_to_acc);
	acc_f->y = (float)(_acc_calm.K[1]*((float)_Mpu.acc.y) - _acc_calm.B[1]*one_g_to_acc);
	acc_f->z = (float)(_acc_calm.K[2]*((float)_Mpu.acc.z) - _acc_calm.B[2]*one_g_to_acc);
}


/* 磁力计校准
 */
void mag_calibration(_F32xyz* mag)
{
	if(get_mag_raw(&_Mag.mag) == true)
	{
		//偏零校准
		mag->x = _mag_calm.x_gain*(_Mag.mag.x - _mag_calm.offset.x);
		mag->y = _mag_calm.y_gain*(_Mag.mag.y - _mag_calm.offset.y);
		mag->z = _mag_calm.z_gain*(_Mag.mag.z - _mag_calm.offset.z);
//		//不校准
//		mag->x = _Mag.mag.x;
//		mag->y = _Mag.mag.y;
//		mag->z = _Mag.mag.z;
	}
}



