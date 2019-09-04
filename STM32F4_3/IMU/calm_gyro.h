#ifndef __CALM_GYRO_H
#define __CALM_GYRO_H

#include "system.h"

#include <stdbool.h>

typedef struct
{
    uint16_t i;
    bool  star_flag;//��ʼ��־λ
    bool  finish_flag;//У׼��ɱ�־λ
    _F32xyz offset;
	_F32xyz offset_f;
}_GYRO_CAL;

extern _GYRO_CAL _gyro_calm;

void gyro_calm(_F32xyz *gyro_in);//������ƫ��У׼

#endif

