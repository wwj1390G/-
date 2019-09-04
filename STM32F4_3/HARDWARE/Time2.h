#ifndef __TIME2_H
#define __TIME2_H
#include "system.h"

typedef struct
{
    float       last_time_us;
    float       now_time_us;
    float       delta_time_us;
    float       delta_time_ms;
	float       delta_time_s;
}_Time_test;   
extern _Time_test IMU_TIM;
extern uint32_t running_tim_cnt;

void time_check(_Time_test *running);

void TIM2_Init(void);




#endif

