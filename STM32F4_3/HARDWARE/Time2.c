#include "Time2.h"

#include "filter.h"



uint32_t running_tim_cnt = 0;
_Time_test run_start = {0};
_Time_test run_stop = {0};

_Time_test IMU_TIM = {0};


/*ϵͳ��ʱ��  2ms
 *84M/840=100Khz�ļ���Ƶ��,10us����һ�Σ�����500��Ϊ5ms
 */
void TIM2_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  ///ʹ��TIM3ʱ��
	
	TIM_TimeBaseInitStructure.TIM_Period = 499;    //�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=839;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE); //����ʱ�������ж�
	TIM_Cmd(TIM2,ENABLE); //ʹ�ܶ�ʱ��
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn; //��ʱ��3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1; //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1; //�����ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

//����ϵͳ����ʱ��
void time_check(_Time_test *running)
{
    running->last_time_us = running->now_time_us;
    running->now_time_us = running_tim_cnt * 840 + TIM2->CNT;
    
    running->delta_time_us = running->now_time_us - running->last_time_us;
    running->delta_time_ms = running->delta_time_us * 0.001f;
	running->delta_time_s = running->delta_time_ms * 0.001f;
}


//��ʱ���жϷ�����//5ms	
void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET) //����ж�
	{
		running_tim_cnt++;
		
		AnalyticData();
		//������&���ٶȼ�����2ms�ɼ�һ��
		acc_calibration(&_Mpu.acc_f);//���ٶȼ�У׼ֵ
		gyro_calibration(&_Mpu.gyro_f);//������У׼ֵ
		//����������10ms�ɼ�һ��
		mag_calibration(&_Mag.mag_f);//�����Ƽ�У׼ֵ

		get_deg_s(&_Mpu.gyro_f,&_Mpu.gyro_deg_s);//_gyro_orig_f --> _Mpu.deg_s
		get_rad_s(&_Mpu.gyro_f,&_Mpu.gyro_rad_s);//_gyro_orig_f --> _Mpu.rad_s

		acc_iir_lpf(&_Mpu.acc_f,&_Mpu._acc_att_lpf,_Mpu.Acc_att_coe);//acc_f ==> _acc_att_lpf
		get_acc_g(&_Mpu._acc_att_lpf,&_Mpu.acc_g);//�õ�_Mpu.acc_g

//		acc_iir_lpf(&_Mpu.acc_f,&_Mpu._acc_fix_lpf,_Mpu.Acc_hight_coe);//�õ�acc_fix_lpf
				
		IMU_UPDATE(&_Att,&_Mpu.acc_f,&_Mpu.gyro_rad_s,&_Mag.mag_f);

		ControllerMode();
		ControllerOperation();
		ControllerOut();
		
		ANO_DT_Send_Status(_Att.pitch,_Att.roll,_Att.yaw,2, 0, 0);

		//��ѹ������8ms�ɼ�һ��
		MS5611_Get_Temp(&_MS5611.temperature);
		MS5611_Get_Pressure(&_MS5611.pressure,&_MS5611.temperature);

		ANO_DT_Send_RCData(_RC.thr*2,_RC.yaw+1500,_RC.rol+1500,_RC.pit+1500,
							0,0,0,0,0,0);
//		ANO_DT_Send_Senser2(_MS5611.pressure,450);//��ѹ���ݺͳ���������
		
		LED_Test();
		
		ANO_DMA_READ_DATA();

	}
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);  //����жϱ�־λ
}




