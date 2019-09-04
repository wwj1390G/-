#include "moter_controller.h"
#include "RC_data.h"
#include "PID.h"
#include "PID_controller.h"
#include "PWM.h"
int lock_flag =0;


/*****************************
		    机头
M1(顺时针)       M4(逆时针)          
	   *     p     *
        *    |    *
         *   |   *
          *  |  *
           * | *
 		    * *
  R--------- *
		    * *
		   *   *
          *     *
         *       *
        *         *
       *           *						 
M2(逆时针)  		 M3(顺时针)  
		    机尾
					 
P、R ：是陀螺仪的方向
*****************************/


void ControllerOut(void)
{
	uint16_t moter1 = 0;
	uint16_t moter2 = 0;
	uint16_t moter3 = 0;
	uint16_t moter4 = 0;
	if(lock_flag == 1)
	{
		if(_RC.thr > 20)//0-1000
		{		
			moter1 = _Thr.control_thr + all.pit_gyro.Output - all.rol_gyro.Output + all.yaw_gyro.Output; 
			moter2 = _Thr.control_thr - all.pit_gyro.Output - all.rol_gyro.Output - all.yaw_gyro.Output;
			moter3 = _Thr.control_thr - all.pit_gyro.Output + all.rol_gyro.Output + all.yaw_gyro.Output;  
			moter4 = _Thr.control_thr + all.pit_gyro.Output + all.rol_gyro.Output - all.yaw_gyro.Output;
		
			moter1 = LimitProcess(moter1,200,850); 
			moter2 = LimitProcess(moter2,200,850); 
			moter3 = LimitProcess(moter3,200,850); 
			moter4 = LimitProcess(moter4,200,850); 
			
		}
		else
		{
			moter1 = _RC.thr;
			moter2 = _RC.thr;
			moter3 = _RC.thr;
			moter4 = _RC.thr;
		
			/*清积分*/
			clear_integral(&all.pit_angle);
			clear_integral(&all.pit_gyro);    
			clear_integral(&all.rol_angle);
			clear_integral(&all.rol_gyro);
			clear_integral(&all.yaw_angle);
			clear_integral(&all.yaw_gyro);
		}
		motorsSetRatio(1, moter1);
		motorsSetRatio(2, moter2);
		motorsSetRatio(3, moter3);
		motorsSetRatio(4, moter4);
	}
	else
	{
		motorsSetRatio(1, 0);
		motorsSetRatio(2, 0);
		motorsSetRatio(3, 0);
		motorsSetRatio(4, 0);
		/*清积分*/
		clear_integral(&all.pit_angle);
		clear_integral(&all.pit_gyro);    
		clear_integral(&all.rol_angle);
		clear_integral(&all.rol_gyro);
		clear_integral(&all.yaw_angle);
		clear_integral(&all.yaw_gyro);
	}
}


