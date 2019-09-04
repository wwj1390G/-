#include "PID.h"


_ALL_PID all = {0};

//pid控制参数
float  controller_parameter[15][5] =
{
    //0.kp   1.ki   2.kd    3.积分限幅    4.pid输出限幅
//上50000转/分钟 电机	
    //姿态外环 
    {1.2,  0.003,   0.01,  300 , 800 },         //pit_angle
    {1.2,  0.003,   0.01,  300 , 800 },         //rol_angle
    {0.001,  0,   0,  300 , 800 },          //yaw_angle

    
    //姿态内环
    {0.710,  0.005,   0.021,  300 , 800 }, //pit_gyro
    {0.710,  0.005,   0.021,  300 , 800 }, //rol_gyro
    {4.2,  10,   0.01,  300 , 800 }, //yaw_gyro
	
	
    //定高
    {0  ,0  ,0  ,200,800},                //acc_high 加速度
    {3.1  ,0  ,0  ,200,800},              //vel_high 速度   内环
    {1.05  ,0  ,0  ,200,800},             //pos_high 位置   外环
    
    //定点x
    {0  ,0  ,0  ,200,800},                //acc_x       
    {0.08  ,0.0  ,0.0  ,200,800},         //vel_x
    {2.2  ,0  ,0  ,200,800},              //pos_x

    //定点y
    {0  ,0  ,0  ,200,800},                //acc_y       
    {0.08  ,0.0  ,0.0  ,200,800},         //vel_y
    {2.2  ,0  ,0  ,200,800},              //pos_y    
};

//pid参数配置
static void pid_init(_PID *controller,uint8_t label)
{
    controller->kp                      = controller_parameter[label][0];
    controller->ki                      = controller_parameter[label][1];
    controller->kd                      = controller_parameter[label][2];
    controller->iLimit             		= controller_parameter[label][3];
    controller->Out_max                 = controller_parameter[label][4];      
}


//pid初始化
void all_pid_init(void)
{
		/*姿态外环 */
    pid_init(&all.pit_angle,0);
    pid_init(&all.rol_angle,1);
    pid_init(&all.yaw_angle,2);
    /*姿态内环*/
    pid_init(&all.pit_gyro,3);
    pid_init(&all.rol_gyro,4);
    pid_init(&all.yaw_gyro,5);

		//定高
    pid_init(&all.acc_high,6);
    pid_init(&all.vel_high,7);
    pid_init(&all.pos_high,8);
  
		//定点x
    pid_init(&all.acc_fix_x,9);
    pid_init(&all.vel_fix_x,10);
    pid_init(&all.pos_fix_x,11);    
    //定点y
    pid_init(&all.acc_fix_y,12);
    pid_init(&all.vel_fix_y,13);
    pid_init(&all.pos_fix_y,14); 

} 

/*PID控制函数
 *expect	  measure	 dt
 *期望值      测量值      单位时间:单位 秒
 */
/************************************
*	对PID的理解：
*	P：比例，最好理解，负反馈，对偏差量的直接控制，偏差越大，负反馈越强烈，使系统快速靠近期望值。
*	   但是，单独P控制的系统会出现超调，导致低频震荡。
*	
*	I：积分，误差累积，当误差非常小时，比例P无法对系统进行有效控制，需要对微小的误差不断累加，将误差放大，
*	   然后再进行控制，因为是对误差进行放大，所以当误差本身就很大时，积分的控制量会巨大，肯定是不可以的。
*	   所以积分要有限制条件，对积分进行限幅。（积分I只是对比例P的辅助调节，要求不高的系统里可以不用积分）
*	
*	D：微分，单位时间的变化量，也就是变化率，变化率越大控制量越大，抑制系统的变化，强迫系统保持当前状态。
* 	  用来抑制比例P的超调缺陷，但是抑制量如果过大，会导致系统高频震荡。
**************************************/
void pid_controller(_PID *pid,float dt)
{
	//误差=期望-实际
	pid->Error  = pid->expect - pid->measure;
	
	//微分=(本次误差-上次误差)/所耗时间
	pid->Deriv  = (pid->Error - pid->PreError) / dt;
	
	//积分=误差*所耗时间
	pid->Integ += pid->Error * dt;
	
	//积分限幅
	if(pid->Integ > pid->iLimit) pid->Integ = pid->iLimit ;
	else if(pid->Integ < -pid->iLimit) pid->Integ = -pid->iLimit;
	
	pid->OutP = pid->kp * pid->Error;//比例控制量输出
	pid->OutI = pid->ki * pid->Integ;//积分控制量输出
	pid->OutD = pid->kd * pid->Deriv;//微分控制量输出
	
	pid->Output = pid->OutP + pid->OutI + pid->OutD;//PID控制量输出
	//输出限幅
	if(pid->Output >  pid->Out_max)   pid->Output =  pid->Out_max;
	if(pid->Output < -pid->Out_max)   pid->Output = -pid->Out_max;
	
	pid->PreError = pid->Error;
}


//清除积分
void clear_integral(_PID *controller)
{
    controller->Integ = 0.0f;
}

