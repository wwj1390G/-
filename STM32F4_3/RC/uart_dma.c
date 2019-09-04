#include "uart_DMA.h"

#define	TX_LEN 32
#define	RX_LEN 32

uint8_t Tx4Buff[TX_LEN] = {0};//�������ݻ�����
uint8_t Rx4Buff[RX_LEN] = {0};//�������ݻ�����
bool USART4_RX_FLAG = 0; //���ڽ��յ����ݱ�־λ
uint16_t USART4_Read_num = 0; //��¼�յ������ݳ���



static void UART_GPIO_Init(void)	
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);//GPIOʹ��
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_UART4); //GPIOA0����ΪUART4
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_UART4); //GPIOA1����ΪUART4
	
	//USART_TX   USART_RX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure);
}
static void UART_NVIC_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel=UART4_IRQn;//����
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
	NVIC_Init(&NVIC_InitStructure);
}

static void DMA_RX_NVIC_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel=DMA1_Stream2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
	NVIC_Init(&NVIC_InitStructure);
}

static void DMA_TX_NVIC_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;	
	
	NVIC_InitStructure.NVIC_IRQChannel=DMA1_Stream4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
	NVIC_Init(&NVIC_InitStructure);
}


static void UART_DMA_RX_Init(void)
{
	DMA_InitTypeDef  DMA_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA2ʱ��ʹ�� 
	
	/*UART4_RX-----ͨ��4��������2*/
	DMA_DeInit(DMA1_Stream2);
	while (DMA_GetCmdStatus(DMA1_Stream2) != DISABLE){}//�ȴ�DMA������ 
	
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //ͨ��ѡ��
	DMA_InitStructure.DMA_BufferSize = RX_LEN;//���ݴ����� 
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//����--���洢��
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// ʹ����ͨģʽ 	
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//�е����ȼ�
		
		
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)Rx4Buff;//DMA �洢��0��ַ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//�洢�����ݳ���:8λ
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//�洢������ģʽ
		
		
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&UART4->DR;//DMA�����ַ
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//�������ݳ���:8λ
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//���������ģʽ
		
	
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//�洢��ͻ�����δ���
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//����ͻ�����δ���
	DMA_Init(DMA1_Stream2,&DMA_InitStructure);//��ʼ��DMA Stream
	
	
	DMA_Cmd(DMA1_Stream2,ENABLE);///ʹ��������
}


static void UART_DMA_TX_Init(void)
{
	DMA_InitTypeDef  DMA_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA2ʱ��ʹ�� 
	
	/*UART4_TX-----ͨ��4��������7*/
	DMA_DeInit(DMA1_Stream4);
	while (DMA_GetCmdStatus(DMA1_Stream4) != DISABLE){}//�ȴ�DMA������ 
	
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //ͨ��ѡ��
	DMA_InitStructure.DMA_BufferSize = TX_LEN;//���ݴ����� 
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//�洢��--������
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// ʹ����ͨģʽ 	
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//�е����ȼ�
		
		
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)Tx4Buff;//DMA �洢��0��ַ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//�洢�����ݳ���:8λ
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//�洢������ģʽ
		
		
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&UART4->DR;//DMA�����ַ
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//�������ݳ���:8λ
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//���������ģʽ
		
	
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//�洢��ͻ�����δ���
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//����ͻ�����δ���
	DMA_Init(DMA1_Stream4,&DMA_InitStructure);//��ʼ��DMA Stream
	
	DMA_Cmd(DMA1_Stream4,ENABLE);///ʹ��������
}


//*******************************************
//����һ��DMA����
//*******************************************
static void DMA_SendOnce(DMA_Stream_TypeDef *DMA_Streamx,uint16_t len)
{
	
	DMA_Cmd(DMA_Streamx, DISABLE); //�ر�DMA����
	
	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE);//�ȴ�DMA������
	DMA_SetCurrDataCounter(DMA_Streamx, len);
	
	DMA_Cmd(DMA_Streamx, ENABLE); //����DMA����
}


/*
*���ڷ��Ͷ���ֽ�
*data��Ҫ���͵����ݣ����ֽ������Ϊ32
*len �����ݵ��ֽ���
*/
void DMA_UART4_SendData(uint8_t *data,uint16_t len)
{
	uint8_t i;
	
	for(i=0;i<len;i++)
	{
		Tx4Buff[i] = *data;
		data++;
	}
	
	DMA_SendOnce(DMA1_Stream4,len);
}


//����1�жϷ������
void UART4_IRQHandler(void)
{
	unsigned char temp;
	//���ڿ����ж�
	if((USART_GetITStatus(UART4,USART_IT_IDLE) != RESET))
	{
		DMA_Cmd(DMA1_Stream2, DISABLE);
		
		USART4_Read_num = RX_LEN - DMA_GetCurrDataCounter(DMA1_Stream2);//��ȡ�յ������ݳ���
		temp = UART4 -> SR;
		temp = UART4 -> DR;//����жϱ�־
		
		USART4_RX_FLAG = 1;//�������׼

		DMA_SetCurrDataCounter(DMA1_Stream2, RX_LEN);  
        DMA_Cmd(DMA1_Stream2, ENABLE); //  ����������������DMA  ��������˹�׼����һ�εĹ���
		
		
	}
}

/*��������ж�
 */
void DMA1_Stream2_IRQHandler(void)
{	
	if(DMA_GetITStatus(DMA1_Stream2,DMA_IT_TCIF2)!=RESET)//��������ж�
	{
		
		DMA_Cmd(DMA1_Stream2, DISABLE);
		DMA_ClearITPendingBit(DMA1_Stream2,DMA_IT_TCIF2);//�����׼λ
		DMA_Cmd(DMA1_Stream2, ENABLE);
		
	}
}

/*��������ж�
 */
void DMA1_Stream4_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_Stream4,DMA_IT_TCIF4)!=RESET)//��������ж�
	{
		DMA_ClearITPendingBit(DMA1_Stream4,DMA_IT_TCIF4);//�����׼λ
	}
}



void DMA_UART4_Init(u32 bound)
{
	/*USART*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);//����ʱ��ʹ��
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate=bound;//������
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//��ʹ��Ӳ����
	USART_InitStructure.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;
	USART_InitStructure.USART_Parity=USART_Parity_No;//�� ��żУ��
	USART_InitStructure.USART_StopBits=USART_StopBits_1;//ֹͣλ 1λ
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;//�ַ�����
	USART_Init(UART4,&USART_InitStructure);
	
	UART_GPIO_Init();
	UART_NVIC_Init();
	USART_ITConfig(UART4,USART_IT_IDLE,ENABLE);//�������ڿ����ж�
	
	USART_Cmd(UART4,ENABLE);//����ʹ��
	
	/*DMA*/
	UART_DMA_TX_Init();
	UART_DMA_RX_Init();
	DMA_RX_NVIC_Init();
	DMA_TX_NVIC_Init();
	
	DMA_ITConfig(DMA1_Stream2,DMA_IT_TC,ENABLE);//����DMA ���� ����ж�
	DMA_ITConfig(DMA1_Stream4,DMA_IT_TC,ENABLE);//����DMA ���� ����ж�
	
	USART_DMACmd(UART4,USART_DMAReq_Rx,ENABLE);//DMA ʹ�ܽ�����
	USART_DMACmd(UART4,USART_DMAReq_Tx,ENABLE);//DMA ʹ�ܷ�����	
	

}

