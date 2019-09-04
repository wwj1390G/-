#ifndef __CLAM_MAG_H
#define __CLAM_MAG_H
#include "system.h"

typedef struct
{
	bool star_flag;	//��ʼУ׼
	bool  finish_flag;//У׼��ɱ�־λ

	_F32xyz Max;
	_F32xyz Min;

	_F32xyz offset;

	int x_gain;
	float y_gain;
	float z_gain;	
}_MAG_CAL;

extern _MAG_CAL _mag_calm ;

void mag_calm(_F32xyz *mag_in);

#endif


