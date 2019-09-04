#ifndef __SPI_MPU_H
#define __SPI_MPU_H
#include "system.h"

typedef enum 
{
    SPI_CLOCK_INITIALIZATON = 0,    // Lowest possible
    SPI_CLOCK_SLOW          = 1,    // ~1 MHz    
    SPI_CLOCK_STANDARD      = 2,    // ~10MHz
    SPI_CLOCK_FAST          = 3,    // ~20MHz
    SPI_CLOCK_ULTRAFAST     = 4     // Highest possible
} SPIClockSpeed_e;


void spi1Init(void);
u8 spi1TransferByte(u8 data);
bool spi1Transfer(uint8_t *out, const uint8_t *in, int len);
void spi1SetSpeed(SPIClockSpeed_e speed);


#endif

