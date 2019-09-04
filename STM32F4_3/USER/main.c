#include "system.h"

uint8_t RC_Data[8] = {0};

int main(void)
{ 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4
	
	delay_init(SystemCoreClock/1000000);  //初始化延时函数
	DMA_USART1_Init(115200);//初始化串口波特率为115200
	DMA_UART4_Init(115200);//F4<==>F1 DMA通讯
	LED_Init();
	
	cppmInit();
	GALV_Init();

	
	while(mpu6000Init() == false);
	while(QMC_Init() == false);
	while(MS5611_init() == false);
	delay_ms(500);
	all_pid_init();
	delay_ms(500);
//	_gyro_calm.star_flag = true;//开机校准陀螺仪
	Flash_Read();	
	
	get_iir_factor(&_Mpu.Acc_att_coe,0.005f,15);	//姿态解算时 加速度 低通系数
	TIM2_Init();
	PWM_Init();
	
#if PRINTF	
	printf("system ok \r\n");
#endif
	while(1)
	{		
		if(running_tim_cnt%2 ==0)
		{
			acc_calm(&_Mpu._acc_att_lpf);
			gyro_calm(&_Mpu.gyro_f);
			mag_calm(&_Mag.mag_f);	

			if(_galv.flag == true)
			{			
				_galv.voltage = _galv.buf[0]*Power_Kv;
				_galv.current = _galv.buf[1]*Power_Ki;
				_galv.flag = false;
			}	
		}
			
	}
}


