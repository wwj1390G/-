#ifndef __RC_DATA_H
#define __RC_DATA_H
#include <stdbool.h>
typedef struct
{
//[0;1000] for thr and [-500;+500] for pit/rol/yaw
	short int thr;//��������
	short int pit;//������
	short int rol;//�����
	short int yaw;//ƫ����
	unsigned	char fly_mode;//ģʽ
	
	unsigned	char CH[10];
	
	unsigned	char signal_lost_flag : 1;//�źŶ�ʧ��־λ
	unsigned	char signal_cnt;//�����źŶ�ʧ����
	
	bool lock;
}_RC_Data;

extern _RC_Data _RC;

void AnalyticData(void);

#endif

