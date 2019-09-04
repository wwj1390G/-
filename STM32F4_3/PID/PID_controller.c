#include "PID_controller.h"
#include "Time2.h"
#include "pid.h"
#include "RC_data.h"


_Throttle _Thr;

_Time_test PID_time;

//外环角度控制器
static void angle_controller(void)
{
    static uint16_t yaw_init_cnt = 0;

    all.rol_angle.measure = _Att.roll; //姿态解算的rol角作为反馈量
    pid_controller(&all.rol_angle, PID_time.delta_time_s);   //外环pid控制器，得到all.rol_angle.out
    all.pit_angle.measure = _Att.pitch; //姿态解算的pit角作为反馈量
    pid_controller(&all.pit_angle, PID_time.delta_time_s);   //外环pid控制器，得到all.pit_angle.out 
    
    if(yaw_init_cnt<300)
        yaw_init_cnt++;
    else
    {
        if(_RC.yaw<=20 || _RC.yaw>=-20)//偏航杆位于中位时，进行双环控制
        {
            if(all.yaw_angle.expect==0)
            {
                all.yaw_angle.expect = _Att.yaw;//最初的偏航角度作为期望值  
            }
            all.yaw_angle.measure = _Att.yaw;//姿态解算的yaw角作为反馈量
            pid_controller(&all.yaw_angle, PID_time.delta_time_s);//外环pid控制器，得到all.yaw_angle.out 
            
            all.yaw_gyro.expect = all.yaw_angle.Output; 
        }
        else//偏航杆打舵，遥感值，作为期望值
        {
            all.yaw_angle.expect = 0;
            all.yaw_gyro.expect = _RC.yaw*5;      
        }        
    }          
}
//内环 角速度控制器
static void gyro_controller(void)
{
    all.pit_gyro.expect = all.pit_angle.Output;//外环输出作为内环期望
    all.pit_gyro.measure = _Mpu.gyro_deg_s.x;//角速度值作为反馈量
    pid_controller(&all.pit_gyro, PID_time.delta_time_s);//pid内环控制
                                                    
    all.rol_gyro.expect = all.rol_angle.Output;
    all.rol_gyro.measure = _Mpu.gyro_deg_s.y;
    pid_controller(&all.rol_gyro, PID_time.delta_time_s);
                                                    
    all.yaw_gyro.measure = _Mpu.gyro_deg_s.z;
    pid_controller(&all.yaw_gyro, PID_time.delta_time_s);
}

//模式
void ControllerMode(void)
{
	all.pit_angle.expect = _RC.pit;//遥控数据作为pit的期望值
	all.rol_angle.expect = _RC.rol;//遥控数据作为rol的期望值
	_Thr.control_thr = _RC.thr;//遥控器传入油门
	
}

//PID运算
void ControllerOperation(void)
{
	time_check(&PID_time);
	angle_controller();//外环 角度
	gyro_controller(); //内环 角速度
}


