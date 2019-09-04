#ifndef __UART_APP_H
#define __UART_APP_H
#include "system.h"

extern uint8_t Tx1Buff[32];//�������ݻ�����
extern uint8_t Rx1Buff[32];//�������ݻ�����
extern bool USART1_RX_FLAG; //���ڽ��յ����ݱ�־λ
extern uint16_t USART1_Read_num ; //�յ������ݳ���


void DMA_UART1_SendData(uint8_t *data,uint16_t len);
void DMA_USART1_Init(u32 bound);

/*
���������������
void Test(void)
{
	if(USART1_RX_FLAG == 1)
	{
		//��������
		DMA_UART1_SendData(Rx1Buff,USART4_Read_num);


		//�����׼λ���Կ�ʼ��һ�ν���
		USART1_RX_FLAG = 0;
	}	
}
*/


#endif

