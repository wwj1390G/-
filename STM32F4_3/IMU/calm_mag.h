#ifndef __CLAM_MAG_H
#define __CLAM_MAG_H
#include "system.h"

typedef struct
{
	bool star_flag;	//开始校准
	bool  finish_flag;//校准完成标志位

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


