#ifndef __PID_H
#define __PID_H

#include "stm32f4xx.h"
typedef struct
{
	float kp;
	float ki;
	float kd;
	
	float expect;//期望值
	float measure;//测量值
	
	float Error;      //本次误差
	float PreError;   //上次误差
	float Deriv;      //微分
	float Integ;      //积分
	float iLimit;     //积分限幅

	float OutP;       //< proportional output (debugging)
	float OutI;       //< integral output (debugging)
	float OutD;       //< derivative output (debugging)
	float Output;     //输出
	float Out_max;		//输出限幅
}_PID;

typedef struct
{
    //姿态外环
    _PID pit_angle;
    _PID rol_angle;
    _PID yaw_angle;
    //姿态内环
    _PID pit_gyro;      
    _PID rol_gyro;
    _PID yaw_gyro;
    //定高
    _PID acc_high;
    _PID vel_high;
    _PID pos_high;
    //定点x
    _PID acc_fix_x;
    _PID vel_fix_x;
    _PID pos_fix_x;
    //定点y
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


