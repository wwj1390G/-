#ifndef __IMU_H
#define __IMU_H
#include "stm32f4xx.h"
#include "my_math.h"

//32768 = 2^15  �����ǵ����ֵռ15λ��32768Ϊ������ֵ  
#define PI                      3.1415926535f
#define gyro_raw_to_deg_s       0.0610351563f   //������ԭʼֵ �� ���ٶ�  ��Ӧ�����Ǿ���2000��/s    1/32768*2000 
#define acc_raw_to_g            0.0023925781f   //������ԭʼֵ �� G  ��Ӧ����������Ϊ8g   8/32768*9.8  

#define angle_to_rad              0.0174532922f   //�Ƕ�ת����  2��/360   ��=3.1415926 
#define rad_to_angle            57.29578049f	  //����ת�Ƕ�   360/2p                 
#define gyro_raw_to_radian_s	(gyro_raw_to_deg_s * angle_to_rad)//raw -> rad/s	ת/��

#define accmax_1g      4096  //32768 / 8g   
#define gravity_mss    9.80665f//�������ٶ�(m/s/s)

#define acc_to_1g      gravity_mss / accmax_1g//���ٶȼ�ԭʼֵ ת G
#define one_g_to_acc   accmax_1g / gravity_mss//G ת ���ٶȼ�ԭʼֵ



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

