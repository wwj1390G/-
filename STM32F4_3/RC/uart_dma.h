#ifndef __UART_DMA_H
#define __UART_DMA_H
#include "system.h"

extern uint8_t Tx4Buff[32];//�������ݻ�����
extern uint8_t Rx4Buff[32];//�������ݻ�����
extern bool USART4_RX_FLAG; //���ڽ��յ����ݱ�־λ
extern uint16_t USART4_Read_num; //�յ������ݳ���

void DMA_UART4_Init(u32 bound);
void DMA_UART4_SendData(uint8_t *data,uint16_t len);

/*
���������������
void Test(void)
{
	if(USART4_RX_FLAG == 1)
	{
		//��������
		DMA_UART4_SendData(Rx4Buff,USART4_Read_num);

		//�����׼λ���Կ�ʼ��һ�ν���
		USART4_RX_FLAG = 0;
	}	
}
*/
#endif

