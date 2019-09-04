#ifndef __CALM_ACC_H
#define __CALM_ACC_H

#include "system.h"


typedef struct
{
//	bool star_flag;//��ʼ��־λ
	bool finish_flag;//���ٶȼ�У׼���
	
	uint8_t  single;//����У׼��־λ
	uint32_t i;
	
	
	uint8_t  single_finish_flag[7];//1-6��У׼��ɱ�־λ
	uint8_t all_finish_flag;//���������ݲɼ���ɱ�־λ
	
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

//���ٶȼ�У׼ 
bool acc_calm(_F32xyz *acc_in);
//���ٶȼ�У׼��ѯ
//void acc_cal_polling(void);
#endif

