#ifndef __CPPM_H
#define __CPPM_H
#include "sys.h"
#include <stdbool.h>

void cppmInit(void);
bool cppmIsAvailible(void);

#define CH_NUM 10	//PPM解析通道数

#define CPPM_TIMER                   TIM9
#define CPPM_TIME_CCn  				 TIM_IT_CC2


#define CPPM_TIMER_RCC               RCC_APB2Periph_TIM9
#define CPPM_GPIO_RCC                RCC_AHB1Periph_GPIOA

#define CPPM_GPIO_PORT               GPIOA
#define CPPM_GPIO_PIN                GPIO_Pin_3
#define CPPM_GPIO_SOURCE             GPIO_PinSource3
#define CPPM_GPIO_AF                 GPIO_AF_TIM9



extern uint16_t ppmData[CH_NUM];


#endif
