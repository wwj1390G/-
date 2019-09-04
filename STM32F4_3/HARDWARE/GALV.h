#ifndef __GALV_H
#define __GALV_H

#include "sys.h"
#include <stdbool.h>

typedef struct
{
	u16 buf[2];//0:电压  1:电流	
	bool flag;
	float voltage;
	float current;
}_GALV;

//硬件上电压电阻 单位 k
#define POWER_R1 10
#define POWER_R2 1
#define POWER_R1R2 POWER_R1+POWER_R2

#define POWER_Rc 0.001f //电流采样电阻单位 Ω


#define	Power_Kv 	0.008862f	//3.3/4096*POWER_R1R2/POWER_R2
#define	Power_Ki	1.611328f	//3.3/4096*2/POWER_Rc


extern _GALV _galv;

void GALV_Init(void);
extern u16 POWER[2];
extern bool ADC_flag;

//void MYDMA_Config(void);

#endif

