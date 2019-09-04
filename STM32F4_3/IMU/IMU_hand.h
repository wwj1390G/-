#ifndef __IMU_HAND_H
#define __IMU_HAND_H
#include "stm32f4xx.h"

typedef struct
{
	short x;
	short y;
	short z;
}_S16xyz;

typedef struct
{
	float x;
	float y;
	float z;
}_F32xyz;


typedef struct
{
	//原始值
	_S16xyz gyro;
	_S16xyz acc;
	
	//偏零值
	_F32xyz gyro_offset;
	_F32xyz mag_offset;
	
	//偏零校准值
	_F32xyz gyro_f;
	_F32xyz acc_f;
	//转换值
	_F32xyz gyro_deg_s;//陀螺仪值（gyro_f）--> deg/s  度/秒
	_F32xyz gyro_rad_s;//陀螺仪值（gyro_f）--> rad/s	转/秒
	_F32xyz acc_g;//加速度值（acc_f）-->转化为 g
	
	//低通滤波系数 
	float Acc_att_coe;	//姿态 加速度
	float Acc_hight_coe;//高度 加速度
	
	//IIR低通滤波后的值
	_F32xyz _acc_att_lpf;//水平方向
	_F32xyz _acc_fix_lpf;//垂直方向
}_MPU;

typedef struct
{
	int32_t	temperature;
	int32_t pressure;
}_PRESSURE;

extern _MPU _Mpu;
extern _PRESSURE _MS5611;




void get_acc_g(_F32xyz *acc_in,_F32xyz *acc_out);
void get_rad_s(_F32xyz *gyro_in,_F32xyz *gyro_out);
void get_deg_s(_F32xyz *gyro_in,_F32xyz *gyro_deg_out);
void gyro_calibration(_F32xyz* gyro_f);
void acc_calibration(_F32xyz* acc_f);
void mag_calibration(_F32xyz* mag);

#endif

