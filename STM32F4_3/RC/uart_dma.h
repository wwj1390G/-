#ifndef __UART_DMA_H
#define __UART_DMA_H
#include "system.h"

extern uint8_t Tx4Buff[32];//发送数据缓存区
extern uint8_t Rx4Buff[32];//接收数据缓存区
extern bool USART4_RX_FLAG; //串口接收到数据标志位
extern uint16_t USART4_Read_num; //收到的数据长度

void DMA_UART4_Init(u32 bound);
void DMA_UART4_SendData(uint8_t *data,uint16_t len);

/*
例：处理接收数据
void Test(void)
{
	if(USART4_RX_FLAG == 1)
	{
		//处理数据
		DMA_UART4_SendData(Rx4Buff,USART4_Read_num);

		//清除标准位可以开始下一次接收
		USART4_RX_FLAG = 0;
	}	
}
*/
#endif

