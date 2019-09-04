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
	//ԭʼֵ
	_S16xyz gyro;
	_S16xyz acc;
	
	//ƫ��ֵ
	_F32xyz gyro_offset;
	_F32xyz mag_offset;
	
	//ƫ��У׼ֵ
	_F32xyz gyro_f;
	_F32xyz acc_f;
	//ת��ֵ
	_F32xyz gyro_deg_s;//������ֵ��gyro_f��--> deg/s  ��/��
	_F32xyz gyro_rad_s;//������ֵ��gyro_f��--> rad/s	ת/��
	_F32xyz acc_g;//���ٶ�ֵ��acc_f��-->ת��Ϊ g
	
	//��ͨ�˲�ϵ�� 
	float Acc_att_coe;	//��̬ ���ٶ�
	float Acc_hight_coe;//�߶� ���ٶ�
	
	//IIR��ͨ�˲����ֵ
	_F32xyz _acc_att_lpf;//ˮƽ����
	_F32xyz _acc_fix_lpf;//��ֱ����
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

