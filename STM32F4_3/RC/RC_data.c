#include "RC_data.h"
#include "cppm.h"
#include "uart_DMA.h"
#include "my_math.h"
#include "moter_controller.h"
_RC_Data _RC = {0};
/*
 * CH0 --- yaw
 * CH1 --- pitch
 * CH2 --- thr
 * CH3 --- roll
 * CH4 --- 
 * CH5 --- 
 * CH6 --- mode
 */

/* 处理遥控数据
 */
void AnalyticData(void)
{	
	
	if(USART4_RX_FLAG == 1)
	{
		//处理数据
		if((Rx4Buff[0] == 0x22) && (Rx4Buff[9] == 0x23))
		{
			for(int i = 0;i<8;i++)
			{
				_RC.CH[i] = Rx4Buff[i+1];
			}
			_RC.yaw = LimitProcess((Rx4Buff[8]-102)*10-500,-500,500);//-500 - 500
			_RC.pit = LimitProcess((Rx4Buff[7]-102)*10-500,-500,500);//-500 - 500
			_RC.thr = LimitProcess((Rx4Buff[6]-102)*10,0,1000);//0 - 1000
			_RC.rol = LimitProcess((Rx4Buff[5]-102)*10-500,-500,500);//-500 - 500
			
			if(_RC.thr < 20)
			{
				if((_RC.pit<-450) && (_RC.rol>450))
				{
					lock_flag = 1;//解锁
					_RC.lock = true;
				}
				if((_RC.pit<-450) && (_RC.rol<-450))
				{
					lock_flag = 0;//上锁
					_RC.lock = true;
				}
			}
		}	
		//清除标准位可以开始下一次接收
		USART4_RX_FLAG = 0;
	}	
}
