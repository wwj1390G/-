#ifndef __LED_H
#define __LED_H
#include "system.h"
#include <stdbool.h>
//LED�˿ڶ���
#define LED1 PCout(0)
#define LED0 PCout(1)
#define LED3 PCout(2)
#define LED2 PCout(3)

void LED_Init(void);//��ʼ��

void LED_Test(void);

#endif
