#ifndef __MS5611_H
#define __MS5611_H

#include "IIC_imit.h"

/*IIC接口*/
#define MS_IIC_Init			imit_IIC_Init//初始化IIC的IO口	
#define MS_IIC_Start		imit_IIC_Start//发送IIC开始信号
#define MS_IIC_Stop			imit_IIC_Stop//发送IIC停止信号
#define MS_IIC_Send_Byte	imit_IIC_Send_Byte//IIC发送一个字节
#define MS_IIC_Read_Byte	imit_IIC_Read_Byte//IIC读取一个字节
#define MS_IIC_Wait_Ack		imit_IIC_Wait_Ack//IIC等待ACK信号
#define MS_IIC_Ack			imit_IIC_Ack//IIC发送ACK信号
#define MS_IIC_NAck			imit_IIC_NAck//IIC不发送ACK信号

/******MS5611指令*************************************************/
#define MS5611_Reset		0x1E//复位指令


//D1 ---- 压力值  OSR为转换速率
#define MS5611_D1_OSR_256	0x40
#define MS5611_D1_OSR_512	0x42
#define MS5611_D1_OSR_1024	0x44
#define MS5611_D1_OSR_2048	0x46
#define MS5611_D1_OSR_4096	0x48

//D2 ---- 温度值
#define MS5611_D2_OSR_256	0x50
#define MS5611_D2_OSR_512	0x52
#define MS5611_D2_OSR_1024	0x54
#define MS5611_D2_OSR_2048	0x56
#define MS5611_D2_OSR_4096	0x58

#define MS5611_ADC_Read		0x00

/*地址0包含工厂数据和设置
 *地址1-6校准系数
 *地址7包含串行代码和CRC
 */
#define MS5611_PROM_Read	0xA0	//0xA0-0xAE  16位地址

/*CSB=1时设备地址为 0xEA
 *CSB=0时设备地址为 0xEE
 */
#define MS5611_Addr		0xEE//设备地址


extern bool MS5611_init(void);
extern void MS5611_Get_Temp(int32_t* Temp);
extern void MS5611_Get_Pressure(int32_t* P,int32_t* Temp);
extern float MS5611_Get_High(int32_t* P,int32_t* Temp);

#endif

