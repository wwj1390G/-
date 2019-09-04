#ifndef __QMC5883L_H
#define __QMC5883L_H

#include "system.h"
#include "IMU_hand.h"


/*IIC�ӿ�*/
#define QMC_IIC_Init			imit_IIC_Init//��ʼ��IIC��IO��	
#define QMC_IIC_Start			imit_IIC_Start//����IIC��ʼ�ź�
#define QMC_IIC_Stop			imit_IIC_Stop//����IICֹͣ�ź�
#define QMC_IIC_Send_Byte		imit_IIC_Send_Byte//IIC����һ���ֽ�
#define QMC_IIC_Read_Byte		imit_IIC_Read_Byte//IIC��ȡһ���ֽ�
#define QMC_IIC_Wait_Ack		imit_IIC_Wait_Ack//IIC�ȴ�ACK�ź�
#define QMC_IIC_Ack				imit_IIC_Ack//IIC����ACK�ź�
#define QMC_IIC_NAck			imit_IIC_NAck//IIC������ACK�ź�



 
//���������ݼĴ���
#define MAG_XOUT_L  0x00	//X���8λ
#define MAG_XOUT_H  0x01	//X���8λ
#define MAG_YOUT_L  0x02	//Y���8λ
#define MAG_YOUT_H  0x03	//Y���8λ
#define MAG_ZOUT_L  0x04	//Z���8λ
#define MAG_ZOUT_H  0x05	//Z���8λ

//����״̬�Ĵ���
#define Addr6  0x06
/*[0] DRDY:��0��: no new data, ��1��: new data is ready
 *[1] OVL: ��0��: normal, 	   ��1��: data overflow
 *[2] DOR: ��0��: normal, 	   ��1��: data skipped for reading
 */

//�¶����ݼĴ���
#define TEMP_L  0x07
#define TEMP_H  0x08

//���ƼĴ���1-2
#define CTRL_REG_1  0x09
/*[7,6]
 *[5,4]	00��2G			01��8G
 *[3,2]	00��10Hz		01��50Hz	02��100Hz	03��200Hz
 *[1,0]	00��standly		01��continuous					
 */
#define CTRL_REG_2 0x0A

//����/�������ڼĴ���
#define REG_11 0x0B
#define REG_12 0x0C
//оƬID�Ĵ���
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
