#include "APP_ANO.h"
#include "PID.h"
#include "led.h"

#include "calm_gyro.h"
#include "calm_acc.h"
#include "calm_mag.h"


//���ݲ�ֺ궨��,�ڷ��ʹ���1�ֽڵ���������ʱ
//����int16��float��,��Ҫ�����ݲ�ֳɵ����ֽڽ��з���
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)	  ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

u8 data_to_send[32];	//�������ݻ���

/*
 *������̬����
 *angle_rol��rol
 *angle_pit��pit
 *angle_yaw��Yaw
 *alt��		�߶�
 *fly_model������ģʽ
 *armed��0������1����
 */
void ANO_DT_Send_Status(float angle_pit,float angle_rol,float angle_yaw,\
								int32_t ALT_USE, u8 fly_model, u8 armed)
{
	u8 _cnt=0;
	int16_t _temp = 0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(angle_rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	int32_t _temp2 = ALT_USE;
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	data_to_send[_cnt++] = fly_model;
	
	data_to_send[_cnt++] = armed;
	
	data_to_send[3] = _cnt-4;//��Ч���ݳ���
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	ANO_SendData(data_to_send, _cnt);
}

/*
 *����ԭʼ9������
 * a_ �����ٶȼ�����
 * g_ ������������
 * m_ ������������
 */
void ANO_DT_Send_Senser( int16_t a_x,int16_t a_y,int16_t a_z,\
								int16_t g_x,int16_t g_y,int16_t g_z,\
								int16_t m_x,int16_t m_y,int16_t m_z)
{
	u8 _cnt=0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	
	_temp = a_x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = g_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = m_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	//////////////////////////////////////////////////////////
	
	ANO_SendData(data_to_send,_cnt);
}

/*��ѹ���ݺͳ���������
 *bar: ��ѹ�Ʋ�õĺ���
 *csb����������õľ���
 */
void ANO_DT_Send_Senser2(int32_t bar,uint16_t csb)
{
	u8 _cnt=0,sum=0,i;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x07;
	data_to_send[_cnt++]=0;
	int32_t		_temp1 = bar*100;
	data_to_send[_cnt++]=BYTE3(_temp1);
	data_to_send[_cnt++]=BYTE2(_temp1);
	data_to_send[_cnt++]=BYTE1(_temp1);
	data_to_send[_cnt++]=BYTE0(_temp1);
	
	uint16_t	_temp2 = csb*100;
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	data_to_send[3] = _cnt-4;
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	//////////////////////////////////////////////////////////
	
	ANO_SendData(data_to_send,_cnt);
}

//***********************************************
//ң��������������
//***********************************************
void ANO_DT_Send_RCData(int16_t thr,int16_t yaw,int16_t rol,int16_t pit,\
								int16_t aux1,int16_t aux2,int16_t aux3,\
								int16_t aux4,int16_t aux5,int16_t aux6)
{
	u8 _cnt=0,sum=0,i;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x03;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(thr);
	data_to_send[_cnt++]=BYTE0(thr);
	
	data_to_send[_cnt++]=BYTE1(yaw);
	data_to_send[_cnt++]=BYTE0(yaw);
	
	data_to_send[_cnt++]=BYTE1(rol);
	data_to_send[_cnt++]=BYTE0(rol);
	
	data_to_send[_cnt++]=BYTE1(pit);
	data_to_send[_cnt++]=BYTE0(pit);
	
	data_to_send[_cnt++]=BYTE1(aux1);
	data_to_send[_cnt++]=BYTE0(aux1);
	
	data_to_send[_cnt++]=BYTE1(aux2);
	data_to_send[_cnt++]=BYTE0(aux2);
	
	data_to_send[_cnt++]=BYTE1(aux3);
	data_to_send[_cnt++]=BYTE0(aux3);
	
	data_to_send[_cnt++]=BYTE1(aux4);
	data_to_send[_cnt++]=BYTE0(aux4);
	
	data_to_send[_cnt++]=BYTE1(aux5);
	data_to_send[_cnt++]=BYTE0(aux5);
	
	data_to_send[_cnt++]=BYTE1(aux6);
	data_to_send[_cnt++]=BYTE0(aux6);

	data_to_send[3] = _cnt-4;
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	//////////////////////////////////////////////////////////
	
	ANO_SendData(data_to_send,_cnt);
}

/*
 *����PID����
 *group��PID����֡(1-6)
 */
static void ANO_DT_Send_PID(u8 group,\
							float p1_p,float p1_i,float p1_d,\
							float p2_p,float p2_i,float p2_d,\
							float p3_p,float p3_i,float p3_d)
{
	u8 _cnt=0,sum=0,i;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10+group-1;
	data_to_send[_cnt++]=0;
	
	
	_temp = (int)(p1_p * 1000);//����1000��
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(p1_i * 1000);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(p1_d * 1000);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (int)(p2_p * 1000);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(p2_i * 1000);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(p2_d * 1000);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (int)(p3_p * 1000);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(p3_i * 1000);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(p3_d * 1000);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;

	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	//////////////////////////////////////////////////////////
	
	ANO_SendData(data_to_send,_cnt);
}

/*
 *У�鷵�� ACK
 *uint8  FREAM_HEAD ֡ͷ
 *uint8  CHECK_SUM ��У��
 */
void ANO_DT_Send_CHECK(uint8_t  FREAM_HEAD,uint8_t  CHECK_SUM)
{
	u8 _cnt=0,sum=0,i;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xEF;
	data_to_send[_cnt++]=0;
	
	
	data_to_send[_cnt++]=FREAM_HEAD;
	data_to_send[_cnt++]=CHECK_SUM;
  
	data_to_send[3] = _cnt-4;//������ݳ���

	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	//////////////////////////////////////////////////////////
	
	ANO_SendData(data_to_send,_cnt);
}



//**************************************
//�����1
//**************************************
static void CMD1(uint8_t data)
{
	switch(data)
	{
		/*ACCУ׼ - - ���ٶȼ�*/
		case 0x01:
		#if PRINTF
				printf("_acc_calm.star_flag = %d\r\n",_acc_calm.star_flag);
		#endif		
		break;
		
		/*GYROУ׼ - - ������*/
		case 0x02:_gyro_calm.star_flag = true;
		#if PRINTF
				printf("_gyro_calm.star_flag = %d\r\n",_gyro_calm.star_flag);
		#endif	
				break;
		
		/*MAGУ׼ - - ������*/
		case 0x04:_mag_calm.star_flag = true;
		#if PRINTF
				printf("_mag_calm.star_flag = %d\r\n",_mag_calm.star_flag);
		#endif	
				break;
		
		/*BAROУ׼ - - ��ѹ��*/
		case 0x05:
				break;
		
		//�˳�����У׼
		case 0x20://printf("Retreat Six sides\r\n");
				break;
		
		//����У׼ ��1��	���泯��
		case 0x21:	
					_acc_calm.single = 1;
		#if PRINTF
					printf("_acc_calm.single = %d \r\n",_acc_calm.single);
		#endif	
				break;
		//����У׼ ��2�� ��ͷ����
		case 0x22:	 _acc_calm.single = 4;
		#if PRINTF
					printf("_acc_calm.single = %d \r\n",_acc_calm.single);
		#endif	
				break;
		//����У׼ ��3�� ��೯��
		case 0x23:	_acc_calm.single = 2;
		#if PRINTF
					printf("_acc_calm.single = %d \r\n",_acc_calm.single);
		#endif	
				break;
		//����У׼ ��4��	�Ҳ೯��	
		case 0x24:	_acc_calm.single = 3;
		#if PRINTF
					printf("_acc_calm.single = %d \r\n",_acc_calm.single);
					#endif	
				break;
		//����У׼ ��5��	��β����
		case 0x25:	_acc_calm.single = 5;

		#if PRINTF
					printf("_acc_calm.single = %d \r\n",_acc_calm.single);
		#endif	
				break;
		//����У׼ ��6��	���泯��
		case 0x26:	_acc_calm.single = 6;
		#if PRINTF
					printf("_acc_calm.single = %d \r\n",_acc_calm.single);
		#endif	
				break;
				
	}
}

//**************************************
//�����2
//**************************************
static void CMD2(uint8_t data)
{
	switch(data)
	{
		//��ȡPID����
		case 0x01:
			/*�����⻷ �ǶȻ�*/
			ANO_DT_Send_PID(1,all.rol_angle.kp, all.rol_angle.ki, all.rol_angle.kd,
												all.pit_angle.kp, all.pit_angle.ki, all.pit_angle.kd,
												all.yaw_angle.kp, all.yaw_angle.ki, all.yaw_angle.kd);delay_ms(5);	
			/*�����ڻ� ���ٶȻ�*/
			ANO_DT_Send_PID(2,all.rol_gyro.kp, all.rol_gyro.ki, all.rol_gyro.kd,
												all.pit_gyro.kp, all.pit_gyro.ki, all.pit_gyro.kd,
												all.yaw_gyro.kp, all.yaw_gyro.ki, all.yaw_gyro.kd);delay_ms(5);	

			ANO_DT_Send_PID(3,all.acc_high.kp, all.acc_high.ki, all.acc_high.kd,
												all.vel_high.kp, all.vel_high.ki, all.vel_high.kd,
												all.pos_high.kp, all.pos_high.ki, all.pos_high.kd);delay_ms(5);

			ANO_DT_Send_PID(4, 0, 0, 0,
							0, 0, 0,
							0, 0, 0);delay_ms(5);

			ANO_DT_Send_PID(5, all.acc_fix_x.kp, all.acc_fix_x.ki, all.acc_fix_x.kd,
												 all.vel_fix_x.kp, all.vel_fix_x.ki, all.vel_fix_x.kd,
												 all.pos_fix_x.kp, all.pos_fix_x.ki, all.pos_fix_x.kd);delay_ms(5);

			ANO_DT_Send_PID(6, all.acc_fix_y.kp, all.acc_fix_y.ki, all.acc_fix_y.kd,
												 all.vel_fix_y.kp, all.vel_fix_y.ki, all.vel_fix_y.kd,
												 all.pos_fix_y.kp, all.pos_fix_y.ki, all.pos_fix_y.kd);
					break;
		
		//��ȡ����ģʽ����
		case 0x02:
				break;
	}
}


//**************************************
//��λ��д��PID1
//**************************************
static void ANO_Write_PID1(uint8_t *rx)//PID �⻷(�ǶȻ�)
{
  all.rol_angle.kp = 0.001f*((rx[4]<<8)|rx[5]);   
  all.rol_angle.ki = 0.001f*((rx[6]<<8)|rx[7]);
  all.rol_angle.kd = 0.001f*((rx[8]<<8)|rx[9]);
  
  all.pit_angle.kp = 0.001f*((rx[10]<<8)|rx[11]);
  all.pit_angle.ki = 0.001f*((rx[12]<<8)|rx[13]);
  all.pit_angle.kd = 0.001f*((rx[14]<<8)|rx[15]);
  
  all.yaw_angle.kp = 0.001f*((rx[16]<<8)|rx[17]);
  all.yaw_angle.ki = 0.001f*((rx[18]<<8)|rx[19]);
  all.yaw_angle.kd = 0.001f*((rx[20]<<8)|rx[21]);
  
  ANO_DT_Send_CHECK(rx[2],rx[22]);
}

//**************************************
//��λ��д��PID2
//**************************************
static void ANO_Write_PID2(uint8_t *rx)//PID �ڻ�(���ʻ�)
{
  all.rol_gyro.kp = 0.001f*((rx[4]<<8)|rx[5]);
  all.rol_gyro.ki = 0.001f*((rx[6]<<8)|rx[7]);
  all.rol_gyro.kd = 0.001f*((rx[8]<<8)|rx[9]);
  
  all.pit_gyro.kp = 0.001f*((rx[10]<<8)|rx[11]);
  all.pit_gyro.ki = 0.001f*((rx[12]<<8)|rx[13]);
  all.pit_gyro.kd = 0.001f*((rx[14]<<8)|rx[15]);
  
  all.yaw_gyro.kp = 0.001f*((rx[16]<<8)|rx[17]);
  all.yaw_gyro.ki = 0.001f*((rx[18]<<8)|rx[19]);
  all.yaw_gyro.kd = 0.001f*((rx[20]<<8)|rx[21]);
  
  ANO_DT_Send_CHECK(rx[2],rx[22]);//���� ACK
}


//**************************************
//��λ��д��PID3
//**************************************
static void ANO_Write_PID3(uint8_t *rx)//�߶�PID
{
	all.acc_high.kp = 0.001f*((rx[4]<<8)|rx[5]);
  all.acc_high.ki = 0.001f*((rx[6]<<8)|rx[7]);
  all.acc_high.kd = 0.001f*((rx[8]<<8)|rx[9]);
  
  all.vel_high.kp = 0.001f*((rx[10]<<8)|rx[11]);
  all.vel_high.ki = 0.001f*((rx[12]<<8)|rx[13]);
  all.vel_high.kd = 0.001f*((rx[14]<<8)|rx[15]);
  
  all.pos_high.kp = 0.001f*((rx[16]<<8)|rx[17]);
  all.pos_high.ki = 0.001f*((rx[18]<<8)|rx[19]);
  all.pos_high.kd = 0.001f*((rx[20]<<8)|rx[21]);
  ANO_DT_Send_CHECK(rx[2],rx[22]);
}

//**************************************
//��λ��д��PID4
//**************************************
static void ANO_Write_PID4(uint8_t *rx)
{
  ANO_DT_Send_CHECK(rx[2],rx[22]);
}

//**************************************
//��λ��д��PID5
//**************************************
static void ANO_Write_PID5(uint8_t *rx)
{
	all.acc_fix_x.kp = 0.001f*((rx[4]<<8)|rx[5]);
	all.acc_fix_x.ki = 0.001f*((rx[6]<<8)|rx[7]);
	all.acc_fix_x.kd = 0.001f*((rx[8]<<8)|rx[9]);

	all.vel_fix_x.kp = 0.001f*((rx[10]<<8)|rx[11]);
	all.vel_fix_x.ki = 0.001f*((rx[12]<<8)|rx[13]);
	all.vel_fix_x.kd = 0.001f*((rx[14]<<8)|rx[15]);

	all.pos_fix_x.kp = 0.001f*((rx[16]<<8)|rx[17]);
	all.pos_fix_x.ki = 0.001f*((rx[18]<<8)|rx[19]);
	all.pos_fix_x.kd = 0.001f*((rx[20]<<8)|rx[21]);
	ANO_DT_Send_CHECK(rx[2],rx[22]);
}

//**************************************
//��λ��д��PID6
//**************************************
static void ANO_Write_PID6(uint8_t *rx)
{
	all.acc_fix_y.kp = 0.001f*((rx[4]<<8)|rx[5]);
	all.acc_fix_y.ki = 0.001f*((rx[6]<<8)|rx[7]);
	all.acc_fix_y.kd = 0.001f*((rx[8]<<8)|rx[9]);

	all.vel_fix_y.kp = 0.001f*((rx[10]<<8)|rx[11]);
	all.vel_fix_y.ki = 0.001f*((rx[12]<<8)|rx[13]);
	all.vel_fix_y.kd = 0.001f*((rx[14]<<8)|rx[15]);

	all.pos_fix_y.kp = 0.001f*((rx[16]<<8)|rx[17]);
	all.pos_fix_y.ki = 0.001f*((rx[18]<<8)|rx[19]);
	all.pos_fix_y.kd = 0.001f*((rx[20]<<8)|rx[21]);
	ANO_DT_Send_CHECK(rx[2],rx[22]);
}


//**************************************
//������λ������������
//**************************************
void ANO_DMA_READ_DATA(void)
{
	uint8_t usart_rx[32];
	
	if(ANO_RX_FLAG==1)//���մ��ڴ���������
	{
		//�������ݣ���ֹ�����ڴ�������б������ݸ���
		for(uint8_t i=0;i<32;i++)
		{
			usart_rx[i]=ANO_RxBuff[i];
		}
		
		
		//����
		if((usart_rx[0]==0XAA)&&(usart_rx[1]==0XAF))
		{
			switch(usart_rx[2])
			{
				case 0x01:CMD1(usart_rx[4]);break;
				case 0x02:CMD2(usart_rx[4]);break;
				case 0x10:ANO_Write_PID1(usart_rx);break;
				case 0x11:ANO_Write_PID2(usart_rx);break;
				case 0x12:ANO_Write_PID3(usart_rx);break;
				case 0x13:ANO_Write_PID4(usart_rx);break;
				case 0x14:ANO_Write_PID5(usart_rx);break;
				case 0x15:ANO_Write_PID6(usart_rx);/*PID_Write_Flash()*/;break;
			}
		}
		ANO_RX_FLAG = 0;
	}
}
/**************************************************/

/**************************************************/
void ANO_DMA_DATA(void)
{
	
	
}

