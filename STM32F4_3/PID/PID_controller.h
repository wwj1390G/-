#ifndef __PID_CONTROLLER_H
#define __PID_CONTROLLER_H
#include "stm32f4xx.h" 

typedef struct
{
	uint16_t control_thr;//��������
	uint16_t altitude_thr;//�߶���������
}_Throttle;


void ControllerMode(void);
void ControllerOperation(void);


extern _Throttle _Thr;


#endif



