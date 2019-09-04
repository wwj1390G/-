#ifndef __MS5611_H
#define __MS5611_H

#include "IIC_imit.h"

/*IIC�ӿ�*/
#define MS_IIC_Init			imit_IIC_Init//��ʼ��IIC��IO��	
#define MS_IIC_Start		imit_IIC_Start//����IIC��ʼ�ź�
#define MS_IIC_Stop			imit_IIC_Stop//����IICֹͣ�ź�
#define MS_IIC_Send_Byte	imit_IIC_Send_Byte//IIC����һ���ֽ�
#define MS_IIC_Read_Byte	imit_IIC_Read_Byte//IIC��ȡһ���ֽ�
#define MS_IIC_Wait_Ack		imit_IIC_Wait_Ack//IIC�ȴ�ACK�ź�
#define MS_IIC_Ack			imit_IIC_Ack//IIC����ACK�ź�
#define MS_IIC_NAck			imit_IIC_NAck//IIC������ACK�ź�

/******MS5611ָ��*************************************************/
#define MS5611_Reset		0x1E//��λָ��


//D1 ---- ѹ��ֵ  OSRΪת������
#define MS5611_D1_OSR_256	0x40
#define MS5611_D1_OSR_512	0x42
#define MS5611_D1_OSR_1024	0x44
#define MS5611_D1_OSR_2048	0x46
#define MS5611_D1_OSR_4096	0x48

//D2 ---- �¶�ֵ
#define MS5611_D2_OSR_256	0x50
#define MS5611_D2_OSR_512	0x52
#define MS5611_D2_OSR_1024	0x54
#define MS5611_D2_OSR_2048	0x56
#define MS5611_D2_OSR_4096	0x58

#define MS5611_ADC_Read		0x00

/*��ַ0�����������ݺ�����
 *��ַ1-6У׼ϵ��
 *��ַ7�������д����CRC
 */
#define MS5611_PROM_Read	0xA0	//0xA0-0xAE  16λ��ַ

/*CSB=1ʱ�豸��ַΪ 0xEA
 *CSB=0ʱ�豸��ַΪ 0xEE
 */
#define MS5611_Addr		0xEE//�豸��ַ


extern bool MS5611_init(void);
extern void MS5611_Get_Temp(int32_t* Temp);
extern void MS5611_Get_Pressure(int32_t* P,int32_t* Temp);
extern float MS5611_Get_High(int32_t* P,int32_t* Temp);

#endif

