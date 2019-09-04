#include "GALV.h"
#include "delay.h"
#include "system.h"

/* ��ѹ -- PA2 --  ADC1 2 3 _IN0
 * ���� -- PA4 --  ADC1 2   _IN4
 * ADC1 ==>  DMA2 --> ͨ��0 --> ������0
 */
 
 //���ɼ�����adcֵ���������б���

_GALV _galv = {0};

static void GALV_GPIO_CONFIG(void)
{
	RCC->AHB1ENR |= 1<<0;//ʹ��GPIOAʱ�� 
	GPIO_Set(GPIOA,GPIO_Pin_2|GPIO_Pin_4,GPIO_MODE_AIN,0,0,GPIO_PUPD_PD);
}

static void GALV_ADC_CONFIG(void)
{
	RCC->APB2ENR |= 1<<8;//ʹ��ADC1ʱ��
	
	RCC->APB2RSTR |= 1<<8; //��λADC
	RCC->APB2RSTR &= ~(1<<8); //��λ����
	
	
	ADC->CCR &= ~(3<<16);
	ADC->CCR |= 1<<16;//4��Ƶ;84/4 = 21MHz
	
	ADC1->CR1 = 0;//CR1��������
	//Ĭ��12λ�ֱ���
	ADC1->CR1 |= 1<<8;//ʹ��ɨ��ģʽ
	
	ADC1->CR2 = 0;//CR2��������
	ADC1->CR2 |= 1<<8;//ʹ��DMAģʽ
	ADC1->CR2 |= 1<<9;//ֻҪ��������ת����DMAʹ�ܣ���ᷢ�� DAM ����
	ADC1->CR2 |= 1<<1;//����ת��ģʽ
	
	ADC1->SQR1 |= 1<<20;//ת������Ϊ2
	ADC1->SQR3 |= 2<<0;//ADC1_CH2 ��1��ת��
	ADC1->SQR3 |= 4<<5;//ADC1_CH4 ��2��ת��	
	
	//����Ĭ���Ҷ���
	ADC1->CR2 |= 1<<0;//ʹ��ADC
	ADC1->CR2 |= 1<<30;//��ʼת��
}

/* ��ʼ��DMA
 * 1��DMA_Streamx: DMA������,DMA1_Stream0~7/DMA2_Stream0~7
 * 2��CHX: DMAͨ��ѡ��,��Χ:0~7
 * 3��PAR: �����ַ
 * 4��MAR: �洢����ַ
 * 5��NDTR: ���ݴ����� 
 * 6��DIR: 0=>���赽�洢��; 1=>�洢��������; 2=>�洢�����洢��; 3=>����
 */
static void GALV_DMA_Init(DMA_Stream_TypeDef *DMA_Streamx,u8 CHX,u32 PAR,u32 MAR,u16 NDTR,u8 DIR)
{
	DMA_TypeDef *DMAx;
	u8 streamx;
	if((u32)DMA_Streamx > (u32)DMA2)//�õ���ǰstream������DMA2����DMA1
	{
		DMAx = DMA2;
		RCC->AHB1ENR |= 1<<22;//DMA2ʱ��ʹ�� 
	}else 
	{
		DMAx=DMA1; 
 		RCC->AHB1ENR |= 1<<21;//DMA1ʱ��ʹ�� 
	}
	while(DMA_Streamx->CR&0X01);//�ȴ�DMA������ 
	
	streamx=(((u32)DMA_Streamx-(u32)DMAx)-0X10)/0X18;					//�õ�streamͨ����
 	if(streamx>=6)DMAx->HIFCR|=0X3D<<(6*(streamx-6)+16);			//���֮ǰ��stream�ϵ������жϱ�־
	else if(streamx>=4)DMAx->HIFCR|=0X3D<<6*(streamx-4);    	//���֮ǰ��stream�ϵ������жϱ�־
	else if(streamx>=2)DMAx->LIFCR|=0X3D<<(6*(streamx-2)+16);	//���֮ǰ��stream�ϵ������жϱ�־
	else DMAx->LIFCR|=0X3D<<6*streamx;												//���֮ǰ��stream�ϵ������жϱ�־
	
	DMA_Streamx->PAR=PAR;	//DMA�����ַ
	DMA_Streamx->M0AR=MAR;	//DMA �洢��0��ַ
	DMA_Streamx->NDTR=NDTR;	//DMA ����������
	
	DMA_Streamx->CR=0;		//��ȫ����λCR�Ĵ���ֵ 
	
	DMA_Streamx->CR &= ~(3<<6);
	DMA_Streamx->CR |= DIR<<6;	//DIR[1:0], ���ݴ��䷽��
	
	
	DMA_Streamx->CR |= 1<<4;		//ʹ��DMA��������ж�
	DMA_Streamx->CR |= 1<<8;		//CIRC=1, ѭ��ģʽ
	DMA_Streamx->CR |= 0<<9;		//PINC=0, �����ַָ��̶�
	DMA_Streamx->CR |= 1<<10;		//MINC=1, �洢������ģʽ,����Ϊ MSIZE ֵ
	DMA_Streamx->CR |= 1<<11;		//PSIZE[1:0]=01, �������ݳ���:16λ
	DMA_Streamx->CR |= 1<<13;		//MSIZE[1:0]=01, �洢�����ݳ���:16λ
	DMA_Streamx->CR |= 1<<16;		//PL[1:0]=01, �е����ȼ�
	DMA_Streamx->CR |= 0<<21;		//PBURST[1:0]=00, ����ͻ�����δ���
	DMA_Streamx->CR |= 0<<23;		//MBURST[1:0]=00, �洢��ͻ�����δ���
	DMA_Streamx->CR |= (u32)CHX<<25;	//ͨ��ѡ��
	//DMA_Streamx->FCR=0X21;	//FIFO���ƼĴ���
	DMA_Streamx->CR|=1<<0;		//ʹ��������������DMA����
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
	
















