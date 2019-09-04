#ifndef __QMC5883L_H
#define __QMC5883L_H

#include "system.h"
#include "IMU_hand.h"


/*IIC接口*/
#define QMC_IIC_Init			imit_IIC_Init//初始化IIC的IO口	
#define QMC_IIC_Start			imit_IIC_Start//发送IIC开始信号
#define QMC_IIC_Stop			imit_IIC_Stop//发送IIC停止信号
#define QMC_IIC_Send_Byte		imit_IIC_Send_Byte//IIC发送一个字节
#define QMC_IIC_Read_Byte		imit_IIC_Read_Byte//IIC读取一个字节
#define QMC_IIC_Wait_Ack		imit_IIC_Wait_Ack//IIC等待ACK信号
#define QMC_IIC_Ack				imit_IIC_Ack//IIC发送ACK信号
#define QMC_IIC_NAck			imit_IIC_NAck//IIC不发送ACK信号



 
//磁力计数据寄存器
#define MAG_XOUT_L  0x00	//X轴低8位
#define MAG_XOUT_H  0x01	//X轴高8位
#define MAG_YOUT_L  0x02	//Y轴低8位
#define MAG_YOUT_H  0x03	//Y轴高8位
#define MAG_ZOUT_L  0x04	//Z轴低8位
#define MAG_ZOUT_H  0x05	//Z轴高8位

//数据状态寄存器
#define Addr6  0x06
/*[0] DRDY:“0”: no new data, “1”: new data is ready
 *[1] OVL: “0”: normal, 	   “1”: data overflow
 *[2] DOR: “0”: normal, 	   “1”: data skipped for reading
 */

//温度数据寄存器
#define TEMP_L  0x07
#define TEMP_H  0x08

//控制寄存器1-2
#define CTRL_REG_1  0x09
/*[7,6]
 *[5,4]	00：2G			01：8G
 *[3,2]	00：10Hz		01：50Hz	02：100Hz	03：200Hz
 *[1,0]	00：standly		01：continuous					
 */
#define CTRL_REG_2 0x0A

//设置/重置周期寄存器
#define REG_11 0x0B
#define REG_12 0x0C
//芯片ID寄存器
#define QMC_ADDR 0x0D			

/*****************************************************/

/*****************************************************/

typedef struct
{
	_S16xyz mag;
	_F32xyz mag_f;
}_MAG;

extern _MAG _Mag;

bool QMC_Init(void);
short MPU_Get_Temperature(void);

bool get_mag_raw(_S16xyz* ma);



#endif
