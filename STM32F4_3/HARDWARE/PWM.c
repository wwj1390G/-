#include "PWM.h"
/*TIM3 -- CH1 -- PC6 --> S1
 *TIM3 -- CH2 -- PC7 --> S2
 *TIM3 -- CH3 -- PC8 --> S3
 *TIM3 -- CH4 -- PC9 --> S4
 *
 *TIM4 -- CH1 -- PB6 --> S5
 *TIM4 -- CH2 -- PB7 --> S6
 *TIM4 -- CH3 -- PB8 --> S7
 *TIM4 -- CH4 -- PB9 --> S8
 */
 
static void TIM3_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM3); //GPIO����Ϊ��ʱ��3
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM3); //GPIO����Ϊ��ʱ��3
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource8,GPIO_AF_TIM3); //GPIO����Ϊ��ʱ��3
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource9,GPIO_AF_TIM3); //GPIO����Ϊ��ʱ��3
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
	GPIO_Init(GPIOC,&GPIO_InitStructure);
}
static void TIM3_PWM_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE); 
	
	TIM_DeInit(TIM3);
	TIM3_GPIO_Init();
	
	TIM_TimeBaseInitStructure.TIM_Period = MOTOR_TIM_PERIOD;			//�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler = MOTOR_TIM_PRESCALE;		//��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;		//���ϼ���ģʽ	
	TIM_TimeBaseInitStructure.TIM_ClockDivision=0; 						//ʱ�ӷ�Ƶ
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0;					//�ظ���������
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);
	
	
	//��ʼ��TIM3 Channel1 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;				//PWMģʽ1
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;	//ʹ�����
	TIM_OCInitStructure.TIM_Pulse=0;							//CCRx
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;		//�͵�ƽ��Ч
	TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Set;	//���иߵ�ƽ
	
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);  	//��ʼ��TIM3 CH1����Ƚ�
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);  	//��ʼ��TIM3 CH2����Ƚ�
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);  	//��ʼ��TIM3 CH3����Ƚ�
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);  	//��ʼ��TIM3 CH4����Ƚ�
	
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR1�ϵ�Ԥװ�ؼĴ���
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR2�ϵ�Ԥװ�ؼĴ���
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR3�ϵ�Ԥװ�ؼĴ���
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR4�ϵ�Ԥװ�ؼĴ���
	
	
	TIM_ARRPreloadConfig(TIM3,ENABLE);//ARPEʹ�� 
	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIM4
}


static void TIM4_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_TIM4); //GPIO����Ϊ��ʱ��4
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_TIM4); //GPIO����Ϊ��ʱ��4
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_TIM4); //GPIO����Ϊ��ʱ��4
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_TIM4); //GPIO����Ϊ��ʱ��4
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
	GPIO_Init(GPIOB,&GPIO_InitStructure);
}
static void TIM4_PWM_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE); 
	TIM_DeInit(TIM4);
	TIM4_GPIO_Init();
	
	TIM_TimeBaseInitStructure.TIM_Period = MOTOR_TIM_PERIOD;			//�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler = MOTOR_TIM_PRESCALE;		//��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;		//���ϼ���ģʽ	
	TIM_TimeBaseInitStructure.TIM_ClockDivision=0; 						//ʱ�ӷ�Ƶ
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0;					//�ظ���������
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);
	
	//��ʼ��TIM4 Channel1 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;				//PWMģʽ1
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;	//ʹ�����
	TIM_OCInitStructure.TIM_Pulse=0;							//CCRx
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;		//�͵�ƽ��Ч
	TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Set;	//���иߵ�ƽ
	
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);  	//��ʼ��TIM4 CH1����Ƚ�
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);  	//��ʼ��TIM4 CH2����Ƚ�
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);  	//��ʼ��TIM4 CH3����Ƚ�
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);  	//��ʼ��TIM4 CH4����Ƚ�
	
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM4��CCR1�ϵ�Ԥװ�ؼĴ���
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM4��CCR2�ϵ�Ԥװ�ؼĴ���
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM4��CCR3�ϵ�Ԥװ�ؼĴ���
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM4��CCR4�ϵ�Ԥװ�ؼĴ���
	
	
	TIM_ARRPreloadConfig(TIM4,ENABLE);//ARPEʹ�� 
	TIM_Cmd(TIM4, ENABLE);  //ʹ��TIM4
}

void PWM_Init(void)
{
	TIM4_PWM_Init();
	TIM3_PWM_Init(); 
}


/*val:0-1000*/
static u16 ratioToCCRx(u16 val)
{
	return (MOTOR_TIM_CNT_FOR_HIGH + val * MOTOR_TIM_CNT_FOR_HIGH/ 1000);//MOTOR_TIM_CNT_FOR_HIGHΪ�̶��ߵ�ƽʱ��ֵ
}

/*���õ��PWMռ�ձ�*/
/*ithrust:0-1000*/
void motorsSetRatio(u16 id, u16 ithrust)
{
	switch(id)
	{
		case 1:TIM3->CCR1 = ratioToCCRx(ithrust);
			break;
		case 2:TIM3->CCR2 = ratioToCCRx(ithrust);
			break;
		case 3:TIM3->CCR3 = ratioToCCRx(ithrust);
			break;
		case 4:TIM3->CCR4 = ratioToCCRx(ithrust);
			break;
		
		case 5:TIM4->CCR1 = ratioToCCRx(ithrust);
			break;
		case 6:TIM4->CCR2 = ratioToCCRx(ithrust);
			break;
		case 7:TIM4->CCR3 = ratioToCCRx(ithrust);
			break;
		case 8:TIM4->CCR4 = ratioToCCRx(ithrust);
			break;
		default: break;
	}
}


