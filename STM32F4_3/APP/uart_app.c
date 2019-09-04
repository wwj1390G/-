#include "uart_app.h"

#define	TX_LEN 32
#define	RX_LEN 32
uint8_t Tx1Buff[TX_LEN] = {0};//发送数据缓存区
uint8_t Rx1Buff[RX_LEN] = {0};//接收数据缓存区
bool USART1_RX_FLAG = 0; //串口接收到数据标志位
uint16_t USART1_Read_num = 0; //记录收到的数据长度


static void UART_GPIO_Init(void)	
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);//GPIO使能
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1
	
	//USART_TX   USART_RX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure);
}
static void UART_NVIC_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel=USART1_IRQn;//串口
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=2;
	NVIC_Init(&NVIC_InitStructure);
}

static void DMA_RX_NVIC_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel=DMA2_Stream2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=2;
	NVIC_Init(&NVIC_InitStructure);
}

static void DMA_TX_NVIC_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;	
	
	NVIC_InitStructure.NVIC_IRQChannel=DMA2_Stream7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=2;
	NVIC_Init(&NVIC_InitStructure);
}


static void UART_DMA_RX_Init(void)
{
	DMA_InitTypeDef  DMA_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);//DMA2时钟使能 
	
	/*UART1_RX-----通道4，数据流2*/
	DMA_DeInit(DMA2_Stream2);
	while (DMA_GetCmdStatus(DMA2_Stream2) != DISABLE){}//等待DMA可配置 
	
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //通道选择
	DMA_InitStructure.DMA_BufferSize = RX_LEN;//数据传输量 
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//外设--》存储器
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// 使用普通模式 	
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//中等优先级
		
		
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)Rx1Buff;//DMA 存储器0地址
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//存储器数据长度:8位
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
		
		
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;//DMA外设地址
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据长度:8位
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
		
	
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
	DMA_Init(DMA2_Stream2,&DMA_InitStructure);//初始化DMA Stream
	
	
	DMA_Cmd(DMA2_Stream2,ENABLE);//使能数据流
}


static void UART_DMA_TX_Init(void)
{
	DMA_InitTypeDef  DMA_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);//DMA2时钟使能 
	
	/*UART1_TX-----通道4，数据流7*/
	DMA_DeInit(DMA2_Stream7);
	while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE){}//等待DMA可配置 
	
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //通道选择
	DMA_InitStructure.DMA_BufferSize = TX_LEN;//数据传输量 
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//存储器--》外设
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// 使用普通模式 	
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//中等优先级
		
		
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)Tx1Buff;//DMA 存储器0地址
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//存储器数据长度:8位
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
		
		
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;//DMA外设地址
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据长度:8位
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
		
	
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
	DMA_Init(DMA2_Stream7,&DMA_InitStructure);//初始化DMA Stream
	
	DMA_Cmd(DMA2_Stream7,ENABLE);///使能数据流
}


//*******************************************
//开启一次DMA传输
//*******************************************
static void DMA_SendOnce(DMA_Stream_TypeDef *DMA_Streamx,uint16_t len)
{
	
	DMA_Cmd(DMA_Streamx, DISABLE); //关闭DMA传输
	
	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE);//等待DMA可配置
	DMA_SetCurrDataCounter(DMA_Streamx, len);
	
	DMA_Cmd(DMA_Streamx, ENABLE); //开启DMA传输
}


/*
*串口发送多个字节
*data：要发送的数据，的字节数最大为32
*len ：数据的字节数
*/
void DMA_UART1_SendData(uint8_t *data,uint16_t len)
{
	uint8_t i;
	
	for(i=0;i<len;i++)
	{
		Tx1Buff[i] = *data;
		data++;
	}
	
	DMA_SendOnce(DMA2_Stream7,len);
}


//串口1中断服务程序
void USART1_IRQHandler(void)
{
	unsigned char temp;
	//串口空闲中断
	if((USART_GetITStatus(USART1,USART_IT_IDLE) != RESET))
	{
		DMA_Cmd(DMA2_Stream2, DISABLE);
		
		USART1_Read_num = RX_LEN - DMA_GetCurrDataCounter(DMA2_Stream2);//获取收到的数据长度
		
		temp = USART1 -> SR;
		temp = USART1 -> DR;//清除中断标志
		
		USART1_RX_FLAG = 1;//接收完标准

		DMA_SetCurrDataCounter(DMA2_Stream2, RX_LEN);  
        DMA_Cmd(DMA2_Stream2, ENABLE); //  这两行是重新设置DMA  让这个搬运工准备下一次的工作
	}
}

/*接收完成中断
 */
void DMA2_Stream2_IRQHandler(void)
{	
	if(DMA_GetITStatus(DMA2_Stream2,DMA_IT_TCIF2)!=RESET)//传输完成中断
	{
		DMA_Cmd(DMA2_Stream2, DISABLE);
		
		DMA_ClearITPendingBit(DMA2_Stream2,DMA_IT_TCIF2);//清除标准位
		
		DMA_Cmd(DMA2_Stream2, ENABLE);
		
	}
}

/*发送完成中断
 */
void DMA2_Stream7_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA2_Stream7,DMA_IT_TCIF7)!=RESET)//传输完成中断
	{
		DMA_ClearITPendingBit(DMA2_Stream7,DMA_IT_TCIF7);//清除标准位
	}
}



void DMA_USART1_Init(u32 bound)
{
	/*USART*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//串口时钟使能
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate=bound;//波特率
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//不使用硬件流
	USART_InitStructure.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;
	USART_InitStructure.USART_Parity=USART_Parity_No;//无 奇偶校验
	USART_InitStructure.USART_StopBits=USART_StopBits_1;//停止位 1位
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;//字符长度
	USART_Init(USART1,&USART_InitStructure);
	
	UART_GPIO_Init();
	UART_NVIC_Init();
	USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);//开启串口空闲中断
	
	USART_Cmd(USART1,ENABLE);//串口使能
	
	/*DMA*/
	UART_DMA_TX_Init();
	UART_DMA_RX_Init();
	DMA_RX_NVIC_Init();
	DMA_TX_NVIC_Init();
	
	DMA_ITConfig(DMA2_Stream2,DMA_IT_TC,ENABLE);//开启DMA 接收 完成中断
	DMA_ITConfig(DMA2_Stream7,DMA_IT_TC,ENABLE);//开启DMA 发送 完成中断
	
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);//DMA 使能接收器
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);//DMA 使能发送器	
	

}

