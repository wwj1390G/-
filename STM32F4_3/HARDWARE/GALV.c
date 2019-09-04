#include "GALV.h"
#include "delay.h"
#include "system.h"

/* 电压 -- PA2 --  ADC1 2 3 _IN0
 * 电流 -- PA4 --  ADC1 2   _IN4
 * ADC1 ==>  DMA2 --> 通道0 --> 数据流0
 */
 
 //将采集到的adc值放入数组中保存

_GALV _galv = {0};

static void GALV_GPIO_CONFIG(void)
{
	RCC->AHB1ENR |= 1<<0;//使能GPIOA时钟 
	GPIO_Set(GPIOA,GPIO_Pin_2|GPIO_Pin_4,GPIO_MODE_AIN,0,0,GPIO_PUPD_PD);
}

static void GALV_ADC_CONFIG(void)
{
	RCC->APB2ENR |= 1<<8;//使能ADC1时钟
	
	RCC->APB2RSTR |= 1<<8; //复位ADC
	RCC->APB2RSTR &= ~(1<<8); //复位结束
	
	
	ADC->CCR &= ~(3<<16);
	ADC->CCR |= 1<<16;//4分频;84/4 = 21MHz
	
	ADC1->CR1 = 0;//CR1设置清零
	//默认12位分辨率
	ADC1->CR1 |= 1<<8;//使能扫描模式
	
	ADC1->CR2 = 0;//CR2设置清零
	ADC1->CR2 |= 1<<8;//使能DMA模式
	ADC1->CR2 |= 1<<9;//只要发生数据转换且DMA使能，便会发出 DAM 请求
	ADC1->CR2 |= 1<<1;//连续转换模式
	
	ADC1->SQR1 |= 1<<20;//转换总数为2
	ADC1->SQR3 |= 2<<0;//ADC1_CH2 第1个转换
	ADC1->SQR3 |= 4<<5;//ADC1_CH4 第2个转换	
	
	//数据默认右对齐
	ADC1->CR2 |= 1<<0;//使能ADC
	ADC1->CR2 |= 1<<30;//开始转换
}

/* 初始化DMA
 * 1、DMA_Streamx: DMA数据流,DMA1_Stream0~7/DMA2_Stream0~7
 * 2、CHX: DMA通道选择,范围:0~7
 * 3、PAR: 外设地址
 * 4、MAR: 存储器地址
 * 5、NDTR: 数据传输量 
 * 6、DIR: 0=>外设到存储器; 1=>存储器到外设; 2=>存储器到存储器; 3=>保留
 */
static void GALV_DMA_Init(DMA_Stream_TypeDef *DMA_Streamx,u8 CHX,u32 PAR,u32 MAR,u16 NDTR,u8 DIR)
{
	DMA_TypeDef *DMAx;
	u8 streamx;
	if((u32)DMA_Streamx > (u32)DMA2)//得到当前stream是属于DMA2还是DMA1
	{
		DMAx = DMA2;
		RCC->AHB1ENR |= 1<<22;//DMA2时钟使能 
	}else 
	{
		DMAx=DMA1; 
 		RCC->AHB1ENR |= 1<<21;//DMA1时钟使能 
	}
	while(DMA_Streamx->CR&0X01);//等待DMA可配置 
	
	streamx=(((u32)DMA_Streamx-(u32)DMAx)-0X10)/0X18;					//得到stream通道号
 	if(streamx>=6)DMAx->HIFCR|=0X3D<<(6*(streamx-6)+16);			//清空之前该stream上的所有中断标志
	else if(streamx>=4)DMAx->HIFCR|=0X3D<<6*(streamx-4);    	//清空之前该stream上的所有中断标志
	else if(streamx>=2)DMAx->LIFCR|=0X3D<<(6*(streamx-2)+16);	//清空之前该stream上的所有中断标志
	else DMAx->LIFCR|=0X3D<<6*streamx;												//清空之前该stream上的所有中断标志
	
	DMA_Streamx->PAR=PAR;	//DMA外设地址
	DMA_Streamx->M0AR=MAR;	//DMA 存储器0地址
	DMA_Streamx->NDTR=NDTR;	//DMA 传输数据量
	
	DMA_Streamx->CR=0;		//先全部复位CR寄存器值 
	
	DMA_Streamx->CR &= ~(3<<6);
	DMA_Streamx->CR |= DIR<<6;	//DIR[1:0], 数据传输方向
	
	
	DMA_Streamx->CR |= 1<<4;		//使能DMA传输完成中断
	DMA_Streamx->CR |= 1<<8;		//CIRC=1, 循环模式
	DMA_Streamx->CR |= 0<<9;		//PINC=0, 外设地址指针固定
	DMA_Streamx->CR |= 1<<10;		//MINC=1, 存储器增量模式,增量为 MSIZE 值
	DMA_Streamx->CR |= 1<<11;		//PSIZE[1:0]=01, 外设数据长度:16位
	DMA_Streamx->CR |= 1<<13;		//MSIZE[1:0]=01, 存储器数据长度:16位
	DMA_Streamx->CR |= 1<<16;		//PL[1:0]=01, 中等优先级
	DMA_Streamx->CR |= 0<<21;		//PBURST[1:0]=00, 外设突发单次传输
	DMA_Streamx->CR |= 0<<23;		//MBURST[1:0]=00, 存储器突发单次传输
	DMA_Streamx->CR |= (u32)CHX<<25;	//通道选择
	//DMA_Streamx->FCR=0X21;	//FIFO控制寄存器
	DMA_Streamx->CR|=1<<0;		//使能数据流，开启DMA传输
}

void GALV_Init(void)
{
	MY_NVIC_Init(3,3,DMA2_Stream0_IRQn,4);
	
	GALV_GPIO_CONFIG();
	GALV_ADC_CONFIG();
	GALV_DMA_Init(DMA2_Stream0,0,(u32)&ADC1->DR,(u32)_galv.buf,2,0);
}


void DMA2_Stream0_IRQHandler(void) 
{
	if (DMA_GetFlagStatus(DMA2_Stream0, DMA_IT_TCIF0) == SET)  
	{	
		_galv.flag = true;
		DMA_ClearFlag(DMA2_Stream0, DMA_IT_TCIF0); 
	}
}
	
















