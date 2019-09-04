#include "led.h" 

#include "calm_gyro.h"
#include "calm_acc.h"
#include "calm_mag.h"
#include "RC_data.h"


void LED_Init(void)
{    	 
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//使能时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化

	GPIO_SetBits(GPIOC,GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_6);
}


typedef enum
{
	led_mode0 = 0,
	led_mode1,
	led_mode2,
	led_mode3,
	led_mode4,
	led_mode5,
	led_mode6,
	led_mode7,
}LED_date;


/* LED闪烁
 * fre：闪烁频率 = fre * 函数调频率
 * num：闪烁次数
 * 返回值为true,则可以结束 闪烁
 *
 * 调用实例
	if(lock_flag == 1)
	{
		flag = LED_twinkle(5,10,1);	
		if(flag == 1)
		{
			lock_flag = 0;
		}
	}
 */
static bool LED_twinkle(int fre,int num,LED_date mode)	
{
	static int fre_f = 0;
	static int num_f = 0;
	bool ret = false;
	
	fre_f++;
	if(fre_f > fre)
	{
		fre_f = 0;
		switch(mode)
		{
			case 0: LED0 = !LED0;
				break;
			
			case 1: LED1 = !LED1;
				break;
			
			case 2: LED2 = !LED2;
				break;
			
			case 3: LED3 = !LED3;
				break;
			
			case 4: LED0 = !LED0;
					LED1 = !LED1;
				break;
			
			case 5: LED1 = !LED1;
					LED2 = !LED2;
				break;
			
			case 6: LED2 = !LED2;
					LED3 = !LED3;
				break;
			
			case 7: LED0 = !LED0;
					LED1 = !LED1;
					LED2 = !LED2;
					LED3 = !LED3;
				break;
			default : break;
		}
		num_f++;
		if(num_f > num) 
		{
			num_f = 0;
			ret = true;
		}
	}
	return ret;
}




void LED_Test(void)
{	
	//acc单面正在采集
	if(_acc_calm.single != 0)
	{
		LED_twinkle(5,20,led_mode0);
	}
	//acc校准完成
	if(_acc_calm.finish_flag == true)
	{
		if(LED_twinkle(5,20,led_mode4))
		{
			_acc_calm.finish_flag = false;
		}
	}
	
	//陀螺仪正在校准
	if(_gyro_calm.star_flag == true)
	{	
		LED_twinkle(5,20,led_mode0);
	}
	//陀螺仪校准完成
	if(_gyro_calm.finish_flag == true)
	{	
		if(LED_twinkle(5,20,led_mode4))
		{
			_gyro_calm.finish_flag = false;
		}
	}
	
	
	//磁力计正在校准
	if(_mag_calm.star_flag == true)
	{
		LED_twinkle(5,20,led_mode0);
	}
	//磁力计校准完成
	if(_mag_calm.finish_flag == true)
	{	
		if(LED_twinkle(5,20,led_mode4))
		{
			_mag_calm.finish_flag = false;
		}
	}
	
	if(_RC.lock == true)//上锁/解锁
	{
		if(LED_twinkle(5,20,led_mode7))
		{
			_RC.lock = false;
		}
	}
}







