#ifndef __RC_DATA_H
#define __RC_DATA_H
#include <stdbool.h>
typedef struct
{
//[0;1000] for thr and [-500;+500] for pit/rol/yaw
	short int thr;//油门数据
	short int pit;//俯仰角
	short int rol;//横滚角
	short int yaw;//偏航角
	unsigned	char fly_mode;//模式
	
	unsigned	char CH[10];
	
	unsigned	char signal_lost_flag : 1;//信号丢失标志位
	unsigned	char signal_cnt;//用于信号丢失计数
	
	bool lock;
}_RC_Data;

extern _RC_Data _RC;

void AnalyticData(void);

#endif

