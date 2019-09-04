#include "cppm.h"

uint16_t ppmData[CH_NUM] = {0};


//84M/168 = 0.5MHz   2us����һ�Σ�5000*2us = 10ms���һ�� 
void cppmInit(void)
{
//ʱ��ʹ��
	RCC_AHB1PeriphClockCmd(CPPM_GPIO_RCC,ENABLE);
	RCC_APB2PeriphClockCmd(CPPM_TIMER_RCC,ENABLE);
	
//����GPIO	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = CPPM_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;		//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�2MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 		//���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;		//����
	GPIO_Init(CPPM_GPIO_PORT,&GPIO_InitStructure);
	
//���ö�ʱ��
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 167;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_Period = 4999;   //�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	TIM_TimeBaseInit(CPPM_TIMER,&TIM_TimeBaseInitStructure);
	
//����GPIO
	GPIO_PinAFConfig(CPPM_GPIO_PORT, CPPM_GPIO_SOURCE, CPPM_GPIO_AF);
	
//��ʼ����ʱ�����벶�����
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2; //ѡ������� IC2ӳ�䵽TI1��
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
	TIM_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 ���������˲��� ���˲�
	TIM_ICInit(CPPM_TIMER, &TIM_ICInitStructure);
		
//�����ж�
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_TIM9_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=5;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	
	
	TIM_ITConfig(CPPM_TIMER,TIM_IT_Update | TIM_IT_CC2,ENABLE);//��������ж� ,����CC2IE�����ж�	

	TIM_Cmd(CPPM_TIMER,ENABLE ); 	//ʹ�ܶ�ʱ��5
}





volatile unsigned char Time_Overflow = 0;//��ʱ���ж��������

//���벶��ֵ(TIM9/TIM14��16λ)
void TIM1_BRK_TIM9_IRQHandler(void)
{
	static bool UP_flag = false;
	static uint16_t last_val = 0;
	static unsigned char num = 0;
//	static unsigned char ppm_head = 0;
	unsigned int ppm;
	
	if(TIM_GetITStatus(CPPM_TIMER, TIM_IT_CC2) != RESET)//�����ж�
	{	
		if(UP_flag == true)//�����½���
		{
			UP_flag = false; 
			ppm = TIM_GetCapture2(CPPM_TIMER) - last_val + Time_Overflow*5000;
			
			if(ppm > 2200)
			{
				num = 0;
			}
			ppmData[num] =  ppm;	
			num++;
			
			TIM_Cmd(CPPM_TIMER,DISABLE ); 	//�رն�ʱ��5
			TIM_SetCounter(CPPM_TIMER,0);	//����������
			TIM_OC2PolarityConfig(CPPM_TIMER,TIM_ICPolarity_Rising); //CC2P=0 ����Ϊ�����ز���
			TIM_Cmd(CPPM_TIMER,ENABLE ); 	//ʹ�ܶ�ʱ��5	
		}
		else
		{
			UP_flag = true;//���������ر�׼λ
			Time_Overflow = 0;
			last_val = TIM_GetCapture2(CPPM_TIMER);//��ȡ��ǰ�Ĳ���ֵ.
			TIM_OC2PolarityConfig(CPPM_TIMER,TIM_ICPolarity_Falling);//CC2P=1 ����Ϊ�½��ز���
		}
		
		
		TIM_ClearITPendingBit(CPPM_TIMER, TIM_IT_CC2);//����жϱ�־λ
	}
	
	if(TIM_GetITStatus(CPPM_TIMER, TIM_IT_Update) != RESET)//����ж�
	{	     
		Time_Overflow++;
		TIM_ClearITPendingBit(CPPM_TIMER, TIM_IT_Update);//����жϱ�־λ
	}	
}
	




