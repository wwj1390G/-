#ifndef __PID_H
#define __PID_H

#include "stm32f4xx.h"
typedef struct
{
	float kp;
	float ki;
	float kd;
	
	float expect;//����ֵ
	float measure;//����ֵ
	
	float Error;      //�������
	float PreError;   //�ϴ����
	float Deriv;      //΢��
	float Integ;      //����
	float iLimit;     //�����޷�

	float OutP;       //< proportional output (debugging)
	float OutI;       //< integral output (debugging)
	float OutD;       //< derivative output (debugging)
	float Output;     //���
	float Out_max;		//����޷�
}_PID;

typedef struct
{
    //��̬�⻷
    _PID pit_angle;
    _PID rol_angle;
    _PID yaw_angle;
    //��̬�ڻ�
    _PID pit_gyro;      
    _PID rol_gyro;
    _PID yaw_gyro;
    //����
    _PID acc_high;
    _PID vel_high;
    _PID pos_high;
    //����x
    _PID acc_fix_x;
    _PID vel_fix_x;
    _PID pos_fix_x;
    //����y
    _PID acc_fix_y;
    _PID vel_fix_y;
    _PID pos_fix_y;
           
}_ALL_PID;

extern _ALL_PID all;

void all_pid_init(void);
void pid_controller(_PID *pid,float dt);
void clear_integral(_PID *controller);

extern float  controller_parameter[15][5];


#endif


