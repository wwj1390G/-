#include "cppm.h"

uint16_t ppmData[CH_NUM] = {0};


//84M/168 = 0.5MHz   2us计数一次，5000*2us = 10ms溢出一次 
void cppmInit(void)
{
//时钟使能
	RCC_AHB1PeriphClockCmd(CPPM_GPIO_RCC,ENABLE);
	RCC_APB2PeriphClockCmd(CPPM_TIMER_RCC,ENABLE);
	
//配置GPIO	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = CPPM_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;		//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度2MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 		//推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;		//上拉
	GPIO_Init(CPPM_GPIO_PORT,&GPIO_InitStructure);
	
//配置定时器
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 167;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_Period = 4999;   //自动重装载值
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	TIM_TimeBaseInit(CPPM_TIMER,&TIM_TimeBaseInitStructure);
	
//复用GPIO
	GPIO_PinAFConfig(CPPM_GPIO_PORT, CPPM_GPIO_SOURCE, CPPM_GPIO_AF);
	
//初始化定时器输入捕获参数
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2; //选择输入端 IC2映射到TI1上
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
	TIM_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
	TIM_ICInit(CPPM_TIMER, &TIM_ICInitStructure);
		
//配置中断
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_TIM9_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=5;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	
	
	TIM_ITConfig(CPPM_TIMER,TIM_IT_Update | TIM_IT_CC2,ENABLE);//允许更新中断 ,允许CC2IE捕获中断	

	TIM_Cmd(CPPM_TIMER,ENABLE ); 	//使能定时器5
}





volatile unsigned char Time_Overflow = 0;//定时器中断溢出次数

//输入捕获值(TIM9/TIM14是16位)
void TIM1_BRK_TIM9_IRQHandler(void)
{
	static bool UP_flag = false;
	static uint16_t last_val = 0;
	static unsigned char num = 0;
//	static unsigned char ppm_head = 0;
	unsigned int ppm;
	
	if(TIM_GetITStatus(CPPM_TIMER, TIM_IT_CC2) != RESET)//捕获中断
	{	
		if(UP_flag == true)//捕获到下降沿
		{
			UP_flag = false; 
			ppm = TIM_GetCapture2(CPPM_TIMER) - last_val + Time_Overflow*5000;
			
			if(ppm > 2200)
			{
				num = 0;
			}
			ppmData[num] =  ppm;	
			num++;
			
			TIM_Cmd(CPPM_TIMER,DISABLE ); 	//关闭定时器5
			TIM_SetCounter(CPPM_TIMER,0);	//计数器清零
			TIM_OC2PolarityConfig(CPPM_TIMER,TIM_ICPolarity_Rising); //CC2P=0 设置为上升沿捕获
			TIM_Cmd(CPPM_TIMER,ENABLE ); 	//使能定时器5	
		}
		else
		{
			UP_flag = true;//捕获到上升沿标准位
			Time_Overflow = 0;
			last_val = TIM_GetCapture2(CPPM_TIMER);//获取当前的捕获值.
			TIM_OC2PolarityConfig(CPPM_TIMER,TIM_ICPolarity_Falling);//CC2P=1 设置为下降沿捕获
		}
		
		
		TIM_ClearITPendingBit(CPPM_TIMER, TIM_IT_CC2);//清除中断标志位
	}
	
	if(TIM_GetITStatus(CPPM_TIMER, TIM_IT_Update) != RESET)//溢出中断
	{	     
		Time_Overflow++;
		TIM_ClearITPendingBit(CPPM_TIMER, TIM_IT_Update);//清除中断标志位
	}	
}
	




