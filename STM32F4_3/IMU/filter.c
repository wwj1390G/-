#include "filter.h"



/*滤波器....*/
//求IIR滤波因子
void get_iir_factor(float *out_factor,float Time, float Cut_Off)
{
	*out_factor = Time /( Time + 1/(2.0f * PI * Cut_Off) );
}

//IIR低通滤波器
void acc_iir_lpf(_F32xyz *acc_in,_F32xyz *acc_out,float lpf_factor)
{
	acc_out->x = acc_out->x + lpf_factor*(acc_in->x - acc_out->x); 
	acc_out->y = acc_out->y + lpf_factor*(acc_in->y - acc_out->y); 
	acc_out->z = acc_out->z + lpf_factor*(acc_in->z - acc_out->z); 
}



//二阶butterworth滤波器-lpf
float butterworth_lpf(float now_input,_Butterworth_data *buffer, _Butterworth_parameter *parameter)
{
    buffer->input_data[2] = now_input;

    /* Butterworth LPF */
    buffer->output_data[2] =   parameter->b[0] * buffer->input_data[2]
                             + parameter->b[1] * buffer->input_data[1]
                             + parameter->b[2] * buffer->input_data[0]
                             - parameter->a[1] * buffer->output_data[1]
                             - parameter->a[2] * buffer->output_data[0];
    /* x(n) 保存 */
    buffer->input_data[0] = buffer->input_data[1];
    buffer->input_data[1] = buffer->input_data[2];
    /* y(n) 保存 */
    buffer->output_data[0] = buffer->output_data[1];
    buffer->output_data[1] = buffer->output_data[2];
    
    return buffer->output_data[2];
}

//加速度计滤波参数
_Butterworth_parameter acc_5hz_parameter =
{
	/*200hz---1hz*/
//  1,   -1.955578240315,   0.9565436765112,
//  0.000241359049042, 0.000482718098084, 0.000241359049042
	/*200hz---2hz*/
//  1,   -1.911197067426,   0.9149758348014,
//  0.0009446918438402,  0.00188938368768,0.0009446918438402
	/*200hz---5hz*/
    1,                  -1.778631777825,    0.8008026466657,
    0.005542717210281,   0.01108543442056,  0.005542717210281
	/*200hz---10hz*/
//    1,   -1.561018075801,   0.6413515380576,
//    0.02008336556421,  0.04016673112842,  0.02008336556421
    /*200hz---15hz*/
//    1,   -1.348967745253,   0.5139818942197,
//    0.04125353724172,  0.08250707448344,  0.04125353724172
    /*200hz---20hz*/
//    1,    -1.14298050254,   0.4128015980962,
//    0.06745527388907,   0.1349105477781,  0.06745527388907
    /*200hz---30hz*/
//    1,  -0.7477891782585,    0.272214937925,
//    0.1311064399166,   0.2622128798333,   0.1311064399166 
}; 

_Butterworth_parameter gyro_30hz_parameter =
					{
						//200hz---30hz
						1,  -0.7477891782585,    0.272214937925,
						0.1311064399166,   0.2622128798333,   0.1311064399166 
					}; 

_Butterworth_data   acc_butter_data[3];
//加速度计巴特沃斯低通滤波
void acc_butterworth_lpf(_F32xyz *acc_in,_F32xyz *acc_out)
{
    acc_out->x = butterworth_lpf(acc_in->x,&acc_butter_data[0],&acc_5hz_parameter);
    acc_out->y = butterworth_lpf(acc_in->y,&acc_butter_data[1],&acc_5hz_parameter);
    acc_out->z = butterworth_lpf(acc_in->z,&acc_butter_data[2],&acc_5hz_parameter);    
}



/******************************************************************/


/*	
	Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏
	R:测量噪声，R增大，动态响应变慢，收敛稳定性变好	
*/
#define KALMAN_Q        0.02
#define KALMAN_R        6.0000
/*           卡尔曼对三个轴加速度进行滤波处理           */
double KalmanFilter_x(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
   kg=p_mid/(p_mid+R); //kg为kalman filter，R为噪声
   x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值
                
   p_now=(1-kg)*p_mid;//最优值对应的covariance       
   p_last = p_now; //更新covariance值
   x_last = x_now; //更新系统状态值
   return x_now;                
 }
double KalmanFilter_y(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
   kg=p_mid/(p_mid+R); //kg为kalman filter，R为噪声
   x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值
                
   p_now=(1-kg)*p_mid;//最优值对应的covariance       
   p_last = p_now; //更新covariance值
   x_last = x_now; //更新系统状态值
   return x_now;                
 }
double KalmanFilter_z(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
   kg=p_mid/(p_mid+R); //kg为kalman filter，R为噪声
   x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值
                
   p_now=(1-kg)*p_mid;//最优值对应的covariance       
   p_last = p_now; //更新covariance值
   x_last = x_now; //更新系统状态值
   return x_now;                
 }








