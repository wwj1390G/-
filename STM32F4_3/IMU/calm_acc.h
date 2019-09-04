#ifndef __CALM_ACC_H
#define __CALM_ACC_H

#include "system.h"


typedef struct
{
//	bool star_flag;//开始标志位
	bool finish_flag;//加速度计校准完成
	
	uint8_t  single;//单面校准标志位
	uint32_t i;
	
	
	uint8_t  single_finish_flag[7];//1-6面校准完成标志位
	uint8_t all_finish_flag;//所有面数据采集完成标志位
	
	_F32xyz samples_sum;
	_F32xyz samples[7];
	
	_F32xyz offset_f;
//	_F32xyz offset_flash_read;
//	_F32xyz offset_flash_write;
	
	_F32xyz scale_f;
//	_F32xyz scale_flash_read;
//	_F32xyz scale_flash_write;
	
	float B[3];
	float K[3];
	
	
	
}_ACC_CAL;



extern _ACC_CAL _acc_calm;

//加速度计校准 
bool acc_calm(_F32xyz *acc_in);
//加速度计校准查询
//void acc_cal_polling(void);
#endif

