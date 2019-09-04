#ifndef __IMU_H
#define __IMU_H
#include "stm32f4xx.h"
#include "my_math.h"

//32768 = 2^15  陀螺仪的输出值占15位，32768为最大输出值  
#define PI                      3.1415926535f
#define gyro_raw_to_deg_s       0.0610351563f   //陀螺仪原始值 变 角速度  对应陀螺仪精度2000°/s    1/32768*2000 
#define acc_raw_to_g            0.0023925781f   //陀螺仪原始值 变 G  对应陀螺仪量程为8g   8/32768*9.8  

#define angle_to_rad              0.0174532922f   //角度转弧度  2π/360   π=3.1415926 
#define rad_to_angle            57.29578049f	  //弧度转角度   360/2p                 
#define gyro_raw_to_radian_s	(gyro_raw_to_deg_s * angle_to_rad)//raw -> rad/s	转/秒

#define accmax_1g      4096  //32768 / 8g   
#define gravity_mss    9.80665f//重力加速度(m/s/s)

#define acc_to_1g      gravity_mss / accmax_1g//加速度计原始值 转 G
#define one_g_to_acc   accmax_1g / gravity_mss//G 转 加速度计原始值



typedef struct
{
	float pitch;
	float roll;
	float yaw;
}_angle;

extern _angle _Att;




				
void IMU_update(_angle* att,\
					_F32xyz* acc,\
					_F32xyz* gyro,\
					_F32xyz* mag) ;
void IMUupdate(	_angle* att,\
				_F32xyz* acc,\
				_F32xyz* gyro,\
				_F32xyz* mag);

void imu_update(_angle* att,\
					_F32xyz* acc,\
					_F32xyz* gyro,\
					_F32xyz* mag) ;



void IMU_UPDATE(_angle* att,\
					_F32xyz* acc,\
					_F32xyz* gyro,\
					_F32xyz* mag);
		
#endif

