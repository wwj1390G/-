#ifndef __PWN_H
#define __PWN_H
#include "system.h"


#define TIM_CLOCK_HZ 				84000000

#define STANDARD_PWM_PERIOD      0.0025// 2.5ms = 400Hz
#define MOTOR_TIM_PRESCALE_RAW   (uint32_t)((TIM_CLOCK_HZ/0xFFFF) * STANDARD_PWM_PERIOD + 1) //+1��ʾ����Ҫ��Ƶ��0xFFFF��ʾ����ֵ
#define MOTOR_TIM_PERIOD 		 (uint32_t)(TIM_CLOCK_HZ * STANDARD_PWM_PERIOD / MOTOR_TIM_PRESCALE_RAW)//��ʱ����װ��ֵ
#define MOTOR_TIM_PRESCALE       (uint16_t)(MOTOR_TIM_PRESCALE_RAW - 1)//��ʱ��Ԥ��Ƶֵ
#define MOTOR_TIM_CNT_FOR_HIGH   (uint32_t)(TIM_CLOCK_HZ * 0.001 / MOTOR_TIM_PRESCALE_RAW)//1000us�ߵ�ƽ�������ֵ

#define Moter_M1 1
#define Moter_M2 2
#define Moter_M3 3
#define Moter_M4 4

#define Moter_M5 5
#define Moter_M6 6
#define Moter_M7 7
#define Moter_M8 8

void PWM_Init(void);
void motorsSetRatio(u16 id, u16 ithrust);

#endif

