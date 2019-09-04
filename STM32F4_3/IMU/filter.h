#ifndef __FILTER_H
#define __FILTER_H
#include "system.h"


typedef struct
{
    float input_data[3];
    float output_data[3];
}_Butterworth_data;

typedef struct
{
    const float a[3];
    const float b[3];
}_Butterworth_parameter;

extern _Butterworth_parameter gyro_30hz_parameter;


//IIRµÍÍ¨ÂË²¨Æ÷
void get_iir_factor(float *out_factor,float Time, float Cut_Off);
void acc_iir_lpf(_F32xyz *acc_in,_F32xyz *acc_out,float lpf_factor);


//¶þ½×butterworthÂË²¨Æ÷-lpf
extern float butterworth_lpf(float now_input,_Butterworth_data *buffer, _Butterworth_parameter *parameter);


double KalmanFilter_x(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R);
double KalmanFilter_y(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R);
double KalmanFilter_z(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R);


#endif

