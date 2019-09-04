#include "Time2.h"

#include "filter.h"



uint32_t running_tim_cnt = 0;
_Time_test run_start = {0};
_Time_test run_stop = {0};

_Time_test IMU_TIM = {0};


/*系统定时器  2ms
 *84M/840=100Khz的计数频率,10us计数一次，计数500次为5ms
 */
void TIM2_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  ///使能TIM3时钟
	
	TIM_TimeBaseInitStructure.TIM_Period = 499;    //自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=839;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE); //允许定时器更新中断
	TIM_Cmd(TIM2,ENABLE); //使能定时器
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn; //定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1; //子优先级1
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

//测试系统运行时间
void time_check(_Time_test *running)
{
    running->last_time_us = running->now_time_us;
    running->now_time_us = running_tim_cnt * 840 + TIM2->CNT;
    
    running->delta_time_us = running->now_time_us - running->last_time_us;
    running->delta_time_ms = running->delta_time_us * 0.001f;
	running->delta_time_s = running->delta_time_ms * 0.001f;
}


//定时器中断服务函数//5ms	
void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET) //溢出中断
	{
		running_tim_cnt++;
		
		AnalyticData();
		//陀螺仪&加速度计数据2ms采集一次
		acc_calibration(&_Mpu.acc_f);//加速度计校准值
		gyro_calibration(&_Mpu.gyro_f);//陀螺仪校准值
		//磁力计数据10ms采集一次
		mag_calibration(&_Mag.mag_f);//磁力计计校准值

		get_deg_s(&_Mpu.gyro_f,&_Mpu.gyro_deg_s);//_gyro_orig_f --> _Mpu.deg_s
		get_rad_s(&_Mpu.gyro_f,&_Mpu.gyro_rad_s);//_gyro_orig_f --> _Mpu.rad_s

		acc_iir_lpf(&_Mpu.acc_f,&_Mpu._acc_att_lpf,_Mpu.Acc_att_coe);//acc_f ==> _acc_att_lpf
		get_acc_g(&_Mpu._acc_att_lpf,&_Mpu.acc_g);//得到_Mpu.acc_g

//		acc_iir_lpf(&_Mpu.acc_f,&_Mpu._acc_fix_lpf,_Mpu.Acc_hight_coe);//得到acc_fix_lpf
				
		IMU_UPDATE(&_Att,&_Mpu.acc_f,&_Mpu.gyro_rad_s,&_Mag.mag_f);

		ControllerMode();
		ControllerOperation();
		ControllerOut();
		
		ANO_DT_Send_Status(_Att.pitch,_Att.roll,_Att.yaw,2, 0, 0);

		//气压计数据8ms采集一次
		MS5611_Get_Temp(&_MS5611.temperature);
		MS5611_Get_Pressure(&_MS5611.pressure,&_MS5611.temperature);

		ANO_DT_Send_RCData(_RC.thr*2,_RC.yaw+1500,_RC.rol+1500,_RC.pit+1500,
							0,0,0,0,0,0);
//		ANO_DT_Send_Senser2(_MS5611.pressure,450);//气压数据和超声波数据
		
		LED_Test();
		
		ANO_DMA_READ_DATA();

	}
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);  //清除中断标志位
}




