#include "IMU.h"
#include "Time2.h"
#include "math.h"

_angle _Att = {0};


#define kp 	    9.0f        //比例增益 控制加速计/磁力仪的收敛速度 
#define ki 	    0.008f     //积分增益 控制陀螺偏差收敛速度
#define dt      IMU_TIM.delta_time_s //采集周期  单位 s
#define halfT   dt*0.5f
//#define dt      0.005f //采集周期  单位 s
//#define halfT	dt/2

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;     //quaternion elements representing theestimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;    //scaled integral error  


/*四元素 解算 欧拉角
 */
void IMU_update(_angle* att,\
					_F32xyz* acc,\
					_F32xyz* gyro,\
					_F32xyz* mag) 
{
	float norm;            
	float hx, hy, hz, bx, bz;            
	float vx, vy, vz, wx, wy, wz; //v*当前姿态计算得来的重力在三轴上的分量            
	float ex, ey, ez;             // auxiliary variables to reduce number of repeated operations             
	
	if(acc->x * acc->y * acc->z == 0)
		return;

	if(mag->x * mag->y * mag->z == 0)
		return;
	//姿态解算时间检测
	time_check(&IMU_TIM);
	
	
	// normalise the measurements            
	norm = sqrt(acc->x*acc->x + acc->y*acc->y + acc->z*acc->z);             
	acc->x = acc->x / norm;            
	acc->y = acc->y / norm;            
	acc->z = acc->z / norm;            
	norm = sqrt(mag->x*mag->x + mag->y*mag->y + mag->z*mag->z);            
	mag->x = mag->x / norm;           
	mag->y = mag->y / norm;            
	mag->z = mag->z / norm;                       
	// compute reference direction of magnetic field           
	hx = 2*mag->x*(0.5f - q2*q2 - q3*q3) + 
		 2*mag->y*(q1*q2 - q0*q3) + 
		 2*mag->z*(q1*q3 + q0*q2);           
	hy = 2*mag->x*(q1*q2 + q0*q3) + 
		 2*mag->y*(0.5f - q1*q1 - q3*q3) + 
		 2*mag->z*(q2*q3 - q0*q1);           
	hz = 2*mag->x*(q1*q3 - q0*q2) + 
		 2*mag->y*(q2*q3 + q0*q1) + 
		 2*mag->z*(0.5f - q1*q1 - q2*q2);                    
	bx = sqrt((hx*hx) + (hy*hy));            
	bz = hz;           
	// estimated direction of gravity and magnetic field (v and w) //参考坐标n系转化到载体坐标b系的用四元数表示的方向余弦矩阵第三列即是。       
	//处理后的重力分量            
	vx = 2*(q1*q3 - q0*q2);            
	vy = 2*(q0*q1 + q2*q3);            
	vz = 1 - 2*q1*q1 - 2*q2*q2;
//	vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;//处理后的mag，反向使用DCM得到 
	
	wx = 2*bx*(0.5f - q2*q2 - q3*q3) + 2*bz*(q1*q3 - q0*q2);            
	wy = 2*bx*(q1*q2 - q0*q3) + 2*bz*(q0*q1 + q2*q3);            
	wz = 2*bx*(q0*q2 + q1*q3) + 2*bz*(0.5f - q1*q1 - q2*q2);            
	/* error is sum of cross product between reference direction of fields and direction measured by sensors 
	体现在加速计补偿和磁力计补偿，因为仅仅依靠加速计补偿没法修正Z轴的变差，所以还需要通过磁力计来修正Z轴。（公式28）。
	《四元数解算姿态完全解析及资料汇总》的作者把这部分理解错了，不是什么叉积的差，而叉积的计算就是这样的。计算方法是公式10。
	*/
	ex = (acc->y*vz - acc->z*vy) + (mag->y*wz - mag->z*wy);           
	ey = (acc->z*vx - acc->x*vz) + (mag->z*wx - mag->x*wz);            
	ez = (acc->x*vy - acc->y*vx) + (mag->x*wy - mag->y*wx);                       
	// integral error scaled integral gain             
	exInt = exInt + ex*ki;//* (1.0f / sampleFreq);            
	eyInt = eyInt + ey*ki;//* (1.0f / sampleFreq);            
	ezInt = ezInt + ez*ki;//* (1.0f / sampleFreq);            
	/* adjusted gyroscope measurements//将误差PI后补偿到陀螺仪，
	即补偿零点漂移。通过调节Kp、Ki两个参数，可以控制加速度计修正陀螺仪积分姿态的速度。（公式16和公式29） 
	*/	
	gyro->x = gyro->x + kp*ex + exInt;            
	gyro->y = gyro->y + kp*ey + eyInt;            
	gyro->z = gyro->z + kp*ez + ezInt;                       
	
	// integrate quaternion rate and normalize //一阶龙格库塔法更新四元数            
	q0 = q0 + (-q1*gyro->x - q2*gyro->y - q3*gyro->z)*halfT;            
	q1 = q1 + ( q0*gyro->x + q2*gyro->z - q3*gyro->y)*halfT;            
	q2 = q2 + ( q0*gyro->y - q1*gyro->z + q3*gyro->x)*halfT;            
	q3 = q3 + ( q0*gyro->z + q1*gyro->y - q2*gyro->x)*halfT;                         
	// normalise quaternion           
	norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);           
	q0 = q0 / norm;           
	q1 = q1 / norm;            
	q2 = q2 / norm;            
	q3 = q3 / norm;
	
	att->pitch =  atan2(2.0f*(q0*q1 + q2*q3),q0*q0 - q1*q1 - q2*q2 + q3*q3)*rad_to_angle;
	att->roll =  asin(2.0f*(q0*q2 - q1*q3))*rad_to_angle;       
    att->yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3 + 1)*rad_to_angle;
//	att->yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3 + 1)* 57.3; // yaw
//	att->roll  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // pitch
//	att->pitch = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll
	
/*	
--------------------- 
版权声明：本文为CSDN博主「_Summer__」的原创文章，遵循CC 4.0 by-sa版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/qq_21842557/article/details/50993809
*/
	
}



void IMUupdate(	_angle* att,\
				_F32xyz* acc,\
				_F32xyz* gyro,\
				_F32xyz* mag)
{
	float norm;
	float hx, hy, hz, bx, bz;
	float wx, wy, wz;
	float vx, vy, vz;
	float ex, ey, ez;

	// 先把这些用得到的值算好
	float q0q0 = q0*q0;
	float q0q1 = q0*q1;
	float q0q2 = q0*q2;
	float q0q3 = q0*q3;
	float q1q1 = q1*q1;
	float q1q2 = q1*q2;
	float q1q3 = q1*q3;
	float q2q2 = q2*q2;
	float q2q3 = q2*q3;
	float q3q3 = q3*q3;

	if(acc->x * acc->y * acc->z == 0)
		return;

	if(mag->x * mag->y * mag->z == 0)
		return;
	//姿态解算时间检测
	time_check(&IMU_TIM);

	norm = sqrt(acc->x * acc->x + acc->y * acc->y + acc->z * acc->z);       //acc数据归一化


	acc->x = acc->x / norm;
	acc->y = acc->y / norm;
	acc->z = acc->z / norm;

	norm = sqrt(mag->x * mag->x + mag->y * mag->y + mag->z * mag->z);       //mag数据归一化
 
	mag->x = mag->x / norm;
	mag->y = mag->y / norm;
	mag->z = mag->z / norm;

	//  mx = 0;
	//  my = 0;
	//  mz = 0;

	hx = 2 * mag->x * (0.5f - q2q2 - q3q3) + 2 * mag->y * (q1q2 - q0q3) + 2 * mag->z * (q1q3 + q0q2);  
	hy = 2 * mag->x * (q1q2 + q0q3) + 2 * mag->y * (0.5f - q1q1 - q3q3) + 2 * mag->z * (q2q3 - q0q1);  
	hz = 2 * mag->x * (q1q3 - q0q2) + 2 * mag->y * (q2q3 + q0q1) + 2 * mag->z * (0.5f - q1q1 -q2q2);          
	bx = sqrt((hx*hx) + (hy*hy));  
	bz = hz;

	// estimated direction of gravity and flux (v and w)              估计重力方向和流量/变迁
	vx = 2*(q1q3 - q0q2);                                             //四元素中xyz的表示
	vy = 2*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3 ;

	wx = 2 * bx * (0.5f - q2q2 - q3q3) + 2 * bz * (q1q3 - q0q2);  
	wy = 2 * bx * (q1q2 - q0q3) + 2 * bz * (q0q1 + q2q3);  
	wz = 2 * bx * (q0q2 + q1q3) + 2 * bz * (0.5f - q1q1 - q2q2); 

	// error is sum of cross product between reference direction of fields and direction measured by sensors
	//  ex = (ay*vz - az*vy) ;                                               //向量外积在相减得到差分就是误差
	//  ey = (az*vx - ax*vz) ;
	//  ez = (ax*vy - ay*vx) ;

	ex = (acc->y*vz - acc->z*vy) + (mag->y*wz - mag->z*wy);  
	ey = (acc->z*vx - acc->x*vz) + (mag->z*wx - mag->x*wz);  
	ez = (acc->x*vy - acc->y*vx) + (mag->x*wy - mag->y*wx);

	exInt = exInt + ex * ki;                                //对误差进行积分
	eyInt = eyInt + ey * ki;
	ezInt = ezInt + ez * ki;

	// adjusted gyroscope measurements
	gyro->x = gyro->x + kp*ex + exInt;                                              //将误差PI后补偿到陀螺仪，即补偿零点漂移
	gyro->y = gyro->y + kp*ey + eyInt;
	gyro->z = gyro->z + kp*ez + ezInt;                                          //这里的gz由于没有观测者进行矫正会产生漂移，表现出来的就是积分自增或自减

	// integrate quaternion rate and normalise                           //四元素的微分方程
	q0 = q0 + (-q1*gyro->x - q2*gyro->y - q3*gyro->z)*halfT;
	q1 = q1 + ( q0*gyro->x + q2*gyro->z - q3*gyro->y)*halfT;
	q2 = q2 + ( q0*gyro->y - q1*gyro->z + q3*gyro->x)*halfT;
	q3 = q3 + ( q0*gyro->z + q1*gyro->y - q2*gyro->x)*halfT;

	// normalise quaternion
	norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);

	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;

	att->pitch =  atan2(2.0f*(q0*q1 + q2*q3),q0*q0 - q1*q1 - q2*q2 + q3*q3)*rad_to_angle;
	att->roll =  asin(2.0f*(q0*q2 - q1*q3))*rad_to_angle;       
    att->yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3 + 1)*rad_to_angle;
}


void imu_update(_angle* att,\
					_F32xyz* acc,\
					_F32xyz* gyro,\
					_F32xyz* mag) 
{
	float norm;
	float hx, hy, hz, bx, bz;
	float wx, wy, wz;
	float vx, vy, vz;
	float ex, ey, ez;
    if(acc->x * acc->y * acc->z==0)
        return;
    
	if(mag->x * mag->y * mag->z == 0)
		return;
	
	
    //姿态解算时间检测
    time_check(&IMU_TIM);
    
    //[ax,ay,az]是机体坐标系下加速度计测得的重力向量(竖直向下)
	norm = invSqrt(acc->x * acc->x + acc->y * acc->y + acc->z * acc->z);
	acc->x = acc->x * norm;
	acc->y = acc->y * norm;
	acc->z = acc->z * norm;
	
	
	norm = sqrt(mag->x * mag->x + mag->y * mag->y + mag->z * mag->z);       //mag数据归一化
	mag->x = mag->x / norm;
	mag->y = mag->y / norm;
	mag->z = mag->z / norm;
	
	hx = 2 * mag->x * (0.5f - q2*q2 - q3*q3) + 2 * mag->y * (q1*q2 - q0*q3) + 2 * mag->z * (q1*q3 + q0*q2);  
	hy = 2 * mag->x * (q1*q2 + q0*q3) + 2 * mag->y * (0.5f - q1*q1 - q3*q3) + 2 * mag->z * (q2*q3 - q0*q1);  
	hz = 2 * mag->x * (q1*q3 - q0*q2) + 2 * mag->y * (q2*q3 + q0*q1) + 2 * mag->z * (0.5f - q1*q1 -q2*q2);          
	bx = sqrt((hx*hx) + (hy*hy));  
	bz = hz;
	

	//VectorA = MatrixC * VectorB
	//VectorA ：参考重力向量转到在机体下的值
	//MatrixC ：地理坐标系转机体坐标系的旋转矩阵  
	//VectorB ：参考重力向量（0,0,1）      
    //[vx,vy,vz]是地理坐标系重力分向量[0,0,1]经过DCM旋转矩阵(C(n->b))计算得到的机体坐标系中的重力向量(竖直向下)    

    vx = 2.0f * (q1*q3 -q0*q2);//Mat.DCM_T[0][2];
    vy = 2.0f * (q2*q3 +q0*q1);//Mat.DCM_T[1][2];
    vz = 1.0f - 2.0f * q1*q1 - 2.0f * q2*q2;//Mat.DCM_T[2][2];
	
	
	wx = 2 * bx * (0.5f - q2*q2 - q3*q3) + 2 * bz * (q1*q3 - q0*q2);  
	wy = 2 * bx * (q1*q2 - q0*q3) + 2 * bz * (q0*q1 + q2*q3);  
	wz = 2 * bx * (q0*q2 + q1*q3) + 2 * bz * (0.5f - q1*q1 - q2*q2); 
    
    //机体坐标系下向量叉乘得到误差向量，误差e就是测量得到的vˉ和预测得到的 v^之间的相对旋转。这里的vˉ就是[ax,ay,az]’,v^就是[vx,vy,vz]’
    //利用这个误差来修正DCM方向余弦矩阵(修正DCM矩阵中的四元素)，这个矩阵的作用就是将b系和n正确的转化直到重合。
    //实际上这种修正方法只把b系和n系的XOY平面重合起来，对于z轴旋转的偏航，加速度计无可奈何，
    //但是，由于加速度计无法感知z轴上的旋转运动，所以还需要用地磁计来进一步补偿。
    //两个向量的叉积得到的结果是两个向量的模与他们之间夹角正弦的乘积a×v=|a||v|sinθ,
    //加速度计测量得到的重力向量和预测得到的机体重力向量已经经过单位化，因而他们的模是1，
    //也就是说它们向量的叉积结果仅与sinθ有关，当角度很小时，叉积结果可以近似于角度成正比。

    ex = acc->y * vz - acc->z * vy + (mag->y*wz - mag->z*wy);
	ey = acc->z * vx - acc->x * vz + (mag->z*wx - mag->x*wz);  
	ez = acc->x * vy - acc->y * vx + (mag->x*wy - mag->y*wx);
 
    //对误差向量进行积分
	exInt = exInt + ex*ki;
	eyInt = eyInt + ey*ki;
	ezInt = ezInt + ez*ki;

    //通过调节Kp、Ki两个参数，可以控制加速度计修正陀螺仪积分姿态的速度。
	gyro->x = gyro->x + kp*ex + exInt;
	gyro->y = gyro->y + kp*ey + eyInt;
	gyro->z = gyro->z + kp*ez + ezInt;

    //一阶龙格库塔法更新四元数 
	q0 = q0 + (-q1 * gyro->x - q2 * gyro->y - q3 * gyro->z)* halfT;
	q1 = q1 + ( q0 * gyro->x + q2 * gyro->z - q3 * gyro->y)* halfT;
	q2 = q2 + ( q0 * gyro->y - q1 * gyro->z + q3 * gyro->x)* halfT;
	q3 = q3 + ( q0 * gyro->z + q1 * gyro->y - q2 * gyro->x)* halfT; 

    //把上述运算后的四元数进行归一化处理。得到了物体经过旋转后的新的四元数。
	norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 * norm;
	q1 = q1 * norm;
	q2 = q2 * norm;
	q3 = q3 * norm;
    
	att->pitch =  atan2(2.0f*(q0*q1 + q2*q3),q0*q0 - q1*q1 - q2*q2 + q3*q3);
	att->roll =  asin(2.0f*(q0*q2 - q1*q3));       
 
    att->yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3 + 1);
//	//z轴角速度积分的偏航角
//	att->yaw += _Mpu.deg_s.z  * dt;

	/*          关于地磁如何进行倾角补  
	 * 参考  http://baike.baidu.com/view/1239157.htm?fr=aladdin 
	 * 公式
	 * XH = x*cos(P)+Y*sin(R)*sin(P)-Z*cos(R)*sin(p)
	 * YH = Y*cos(R)+Z*sin(R)
	 */
//	Xr = mag->x * COS(att->pitch/angle_to_rad)
//		+ mag->y * SIN(-att->pitch/angle_to_rad) * SIN(-att->roll/angle_to_rad)
//		- mag->z * COS(att->roll/angle_to_rad) * SIN(-att->pitch/angle_to_rad);
//	
//	Yr = mag->y * COS(att->roll/angle_to_rad) + mag->z * SIN(-att->roll/angle_to_rad);

//	att->yaw = atan2((double)Yr,(double)Xr) * rad_to_angle; // yaw 

	att->yaw *= rad_to_angle;
	att->roll *= rad_to_angle;
	att->pitch *= rad_to_angle;
	
}

void IMU_UPDATE(_angle* att,\
					_F32xyz* acc,\
					_F32xyz* gyro,\
					_F32xyz* mag)
{
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;
	
 
    if(acc->x*acc->y*acc->z==0)
        return;
	
	if(mag->x * mag->y * mag->z == 0)
		return;
    
    //姿态解算时间检测
    time_check(&IMU_TIM);
    
    //[ax,ay,az]是机体坐标系下加速度计测得的重力向量(竖直向下)
	norm = invSqrt(acc->x*acc->x + acc->y*acc->y + acc->z*acc->z);
	acc->x = acc->x * norm;
	acc->y = acc->y * norm;
	acc->z = acc->z * norm;

	//VectorA = MatrixC * VectorB
	//VectorA ：参考重力向量转到在机体下的值
	//MatrixC ：地理坐标系转机体坐标系的旋转矩阵  
	//VectorB ：参考重力向量（0,0,1）      
    //[vx,vy,vz]是地理坐标系重力分向量[0,0,1]经过DCM旋转矩阵(C(n->b))计算得到的机体坐标系中的重力向量(竖直向下)    

    vx = 2.0f * (q1*q3 -q0*q2); //Mat.DCM_T[0][2];
    vy = 2.0f * (q2*q3 +q0*q1); //Mat.DCM_T[1][2];
    vz = 1.0f - 2.0f * q1*q1 - 2.0f * q2*q2;  //Mat.DCM_T[2][2];
    
    //机体坐标系下向量叉乘得到误差向量，误差e就是测量得到的vˉ和预测得到的 v^之间的相对旋转。这里的vˉ就是[ax,ay,az]’,v^就是[vx,vy,vz]’
    //利用这个误差来修正DCM方向余弦矩阵(修正DCM矩阵中的四元素)，这个矩阵的作用就是将b系和n正确的转化直到重合。
    //实际上这种修正方法只把b系和n系的XOY平面重合起来，对于z轴旋转的偏航，加速度计无可奈何，
    //但是，由于加速度计无法感知z轴上的旋转运动，所以还需要用地磁计来进一步补偿。
    //两个向量的叉积得到的结果是两个向量的模与他们之间夹角正弦的乘积a×v=|a||v|sinθ,
    //加速度计测量得到的重力向量和预测得到的机体重力向量已经经过单位化，因而他们的模是1，
    //也就是说它们向量的叉积结果仅与sinθ有关，当角度很小时，叉积结果可以近似于角度成正比。

	ex = acc->y*vz - acc->z*vy;
	ey = acc->z*vx - acc->x*vz;
	ez = acc->x*vy - acc->y*vx;
 
    //对误差向量进行积分
	exInt = exInt + ex*ki*dt;
	eyInt = eyInt + ey*ki*dt;
	ezInt = ezInt + ez*ki*dt;

    //通过调节Kp、Ki两个参数，可以控制加速度计修正陀螺仪积分姿态的速度。
	gyro->x = gyro->x + kp*ex + exInt;
	gyro->y = gyro->y + kp*ey + eyInt;
	gyro->z = gyro->z + kp*ez + ezInt;

    //一阶龙格库塔法更新四元数 
	q0 = q0 + (-q1*gyro->x - q2*gyro->y - q3*gyro->z)* halfT;
	q1 = q1 + ( q0*gyro->x + q2*gyro->z - q3*gyro->y)* halfT;
	q2 = q2 + ( q0*gyro->y - q1*gyro->z + q3*gyro->x)* halfT;
	q3 = q3 + ( q0*gyro->z + q1*gyro->y - q2*gyro->x)* halfT; 

    //把上述运算后的四元数进行归一化处理。得到了物体经过旋转后的新的四元数。
	norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 * norm;
	q1 = q1 * norm;
	q2 = q2 * norm;
	q3 = q3 * norm;
    
	att->pitch =  atan2(2.0f*(q0*q1 + q2*q3),q0*q0 - q1*q1 - q2*q2 + q3*q3) * rad_to_angle;
	att->roll =  asin(2.0f*(q0*q2 - q1*q3)) * rad_to_angle;       
 
    //z轴角速度积分的偏航角
    att->yaw += _Mpu.gyro_deg_s.z  * IMU_TIM.delta_time_s;
}






