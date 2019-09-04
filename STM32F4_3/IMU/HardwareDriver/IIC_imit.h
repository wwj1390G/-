#ifndef __IIC_IMIT_H
#define __IIC_IMIT_H
#include "system.h"
	   		   
//IO��������
#define SDA_IN()  {GPIOB->MODER&=~(3<<(1*2));GPIOB->MODER|=0<<1*2;}	//PB1����ģʽ
#define SDA_OUT() {GPIOB->MODER&=~(3<<(1*2));GPIOB->MODER|=1<<1*2;} //PB1���ģʽ
//IO��������	 
#define IIC_SCL    PBout(0) //SCL
#define IIC_SDA    PBout(1) //SDA	 
#define READ_SDA   PBin(1)  //����SDA 


//IIC���в�������
extern void imit_IIC_Init(void);                //��ʼ��IIC��IO��				 
extern void imit_IIC_Start(void);				//����IIC��ʼ�ź�
extern void imit_IIC_Stop(void);	  			//����IICֹͣ�ź�
extern void imit_IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
extern u8 imit_IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
extern u8 imit_IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
extern void imit_IIC_Ack(void);					//IIC����ACK�ź�
extern void imit_IIC_NAck(void);				//IIC������ACK�ź�


#endif

