#include "PID_controller.h"
#include "Time2.h"
#include "pid.h"
#include "RC_data.h"


_Throttle _Thr;

_Time_test PID_time;

//�⻷�Ƕȿ�����
static void angle_controller(void)
{
    static uint16_t yaw_init_cnt = 0;

    all.rol_angle.measure = _Att.roll; //��̬�����rol����Ϊ������
    pid_controller(&all.rol_angle, PID_time.delta_time_s);   //�⻷pid���������õ�all.rol_angle.out
    all.pit_angle.measure = _Att.pitch; //��̬�����pit����Ϊ������
    pid_controller(&all.pit_angle, PID_time.delta_time_s);   //�⻷pid���������õ�all.pit_angle.out 
    
    if(yaw_init_cnt<300)
        yaw_init_cnt++;
    else
    {
        if(_RC.yaw<=20 || _RC.yaw>=-20)//ƫ����λ����λʱ������˫������
        {
            if(all.yaw_angle.expect==0)
            {
                all.yaw_angle.expect = _Att.yaw;//�����ƫ���Ƕ���Ϊ����ֵ  
            }
            all.yaw_angle.measure = _Att.yaw;//��̬�����yaw����Ϊ������
            pid_controller(&all.yaw_angle, PID_time.delta_time_s);//�⻷pid���������õ�all.yaw_angle.out 
            
            all.yaw_gyro.expect = all.yaw_angle.Output; 
        }
        else//ƫ���˴�棬ң��ֵ����Ϊ����ֵ
        {
            all.yaw_angle.expect = 0;
            all.yaw_gyro.expect = _RC.yaw*5;      
        }        
    }          
}
//�ڻ� ���ٶȿ�����
static void gyro_controller(void)
{
    all.pit_gyro.expect = all.pit_angle.Output;//�⻷�����Ϊ�ڻ�����
    all.pit_gyro.measure = _Mpu.gyro_deg_s.x;//���ٶ�ֵ��Ϊ������
    pid_controller(&all.pit_gyro, PID_time.delta_time_s);//pid�ڻ�����
                                                    
    all.rol_gyro.expect = all.rol_angle.Output;
    all.rol_gyro.measure = _Mpu.gyro_deg_s.y;
    pid_controller(&all.rol_gyro, PID_time.delta_time_s);
                                                    
    all.yaw_gyro.measure = _Mpu.gyro_deg_s.z;
    pid_controller(&all.yaw_gyro, PID_time.delta_time_s);
}

//ģʽ
void ControllerMode(void)
{
	all.pit_angle.expect = _RC.pit;//ң��������Ϊpit������ֵ
	all.rol_angle.expect = _RC.rol;//ң��������Ϊrol������ֵ
	_Thr.control_thr = _RC.thr;//ң������������
	
}

//PID����
void ControllerOperation(void)
{
	time_check(&PID_time);
	angle_controller();//�⻷ �Ƕ�
	gyro_controller(); //�ڻ� ���ٶ�
}


