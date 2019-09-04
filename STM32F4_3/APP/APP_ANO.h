#ifndef __APP_ANO_H
#define __APP_ANO_H
#include "system.h"


//发送数据函数
void ANO_DMA_SEND_DATA(void);//向上位机数据

void ANO_DT_Send_Status(float angle_pit,float angle_rol,float angle_yaw,\
								int32_t ALT_USE, u8 fly_model, u8 armed);//发送姿态数据
void ANO_DT_Send_Senser( int16_t a_x,int16_t a_y,int16_t a_z,\
								int16_t g_x,int16_t g_y,int16_t g_z,\
								int16_t m_x,int16_t m_y,int16_t m_z);//发送原始9轴数据
void ANO_DT_Send_Senser2(int32_t bar,uint16_t csb);//气压数据和超声波数据
void ANO_DT_Send_RCData(int16_t thr,int16_t yaw,int16_t rol,int16_t pit,\
								int16_t aux1,int16_t aux2,int16_t aux3,\
								int16_t aux4,int16_t aux5,int16_t aux6);//遥控器传来的数据

void ANO_DT_Send_CHECK(uint8_t  FREAM_HEAD,uint8_t  CHECK_SUM);//校验返回 ACK



//接收数据函数
void ANO_DMA_READ_DATA(void);//解析串口数据
//void CMD1(uint8_t data);
//void CMD2(uint8_t data);
//void ANO_Write_PID1(uint8_t *rx);
//void ANO_Write_PID2(uint8_t *rx);
//void ANO_Write_PID3(uint8_t *rx);
//void ANO_Write_PID4(uint8_t *rx);
//void ANO_Write_PID5(uint8_t *rx);
//void ANO_Write_PID6(uint8_t *rx);

extern u8 data_to_send[32];	//发送数据缓存

#define ANO_SendData 	DMA_UART1_SendData

#define ANO_TxBuff 		Tx1Buff
#define ANO_RxBuff		Rx1Buff
#define ANO_RX_FLAG		USART1_RX_FLAG
#define ANO_RX_Len		USART1_Read_num

void ANO_DMA_DATA(void);

#endif


