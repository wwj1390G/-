#include "MS5611.h"
/*[0]�������ݺ�����
 *[1-6]У׼ϵ��
 *[7]���д����CRC
 */
uint16_t PROM_C[8] = {0};

/*��λMS5611
 */
static void MS561101BA_RESET(void)
{
	MS_IIC_Start();
	MS_IIC_Send_Byte(MS5611_Addr);
	MS_IIC_Wait_Ack();
	MS_IIC_Send_Byte(MS5611_Reset);
	MS_IIC_Wait_Ack();
	MS_IIC_Stop();
}

/*��ȡPROM
 *����false��ʧ��
 *����true���ɹ�
 */
static bool Read_PROM(void)
{
	int i = 0;
	uint8_t d1,d2;
	for(i=0;i<8;i++)
	{
		MS_IIC_Start();
		MS_IIC_Send_Byte(MS5611_Addr);//������ַ+д����
		if(MS_IIC_Wait_Ack())
		{
			MS_IIC_Stop();	
			return false;
		}
		MS_IIC_Send_Byte(MS5611_PROM_Read + 2*i);
		if(MS_IIC_Wait_Ack())
		{
			MS_IIC_Stop();	
			return false;
		}
		MS_IIC_Stop();

		MS_IIC_Start();
		MS_IIC_Send_Byte(MS5611_Addr+1);//������ַ+������
		if(MS_IIC_Wait_Ack())
		{
			MS_IIC_Stop();	
			return false;
		}
		d1 = MS_IIC_Read_Byte(1);
		d2 = MS_IIC_Read_Byte(0);
		MS_IIC_Stop();
		
		PROM_C[i] = (uint16_t)d1<<8 | d2;
	}
	return true;
}



/*MS5611��ȡת������
 */
static uint32_t MS5611_Get_Conversion(uint8_t cmd)
{
	u8 temp[3] = {0};
	MS_IIC_Start();
	MS_IIC_Send_Byte(MS5611_Addr);//������ַ+д����
	MS_IIC_Wait_Ack();
	MS_IIC_Send_Byte(cmd);//дת������
	MS_IIC_Wait_Ack();
	MS_IIC_Stop();
	
	delay_ms(10);
	
	MS_IIC_Start();
	MS_IIC_Send_Byte(MS5611_Addr);//������ַ+д����
	MS_IIC_Wait_Ack();
	MS_IIC_Send_Byte(MS5611_ADC_Read);//ADC read sequence
	MS_IIC_Wait_Ack();
	MS_IIC_Stop();
	
	MS_IIC_Start();
	MS_IIC_Send_Byte(MS5611_Addr+1);//������ַ+������
	MS_IIC_Wait_Ack();
	temp[0] = MS_IIC_Read_Byte(1);  //��ACK�Ķ�����  bit 23-16
	temp[1] = MS_IIC_Read_Byte(1);  //��ACK�Ķ�����  bit 15-8
	temp[2] = MS_IIC_Read_Byte(0);  //��NACK�Ķ����� bit 7-0
	MS_IIC_Stop();
	
	return ((uint32_t)temp[0]<<16 | (uint32_t)temp[1]<<8 | temp[2]);
	
}



/*��ʼ��MS5611
 *����false��ʧ��
 *����true���ɹ�
 */
bool MS5611_init(void)
{	 
	bool ret = false;
	
	MS_IIC_Init();
	
	MS561101BA_RESET();	 
	delay_ms(100);
	MS561101BA_RESET();//��λMSC
	delay_ms(25);  
	ret = Read_PROM();//��ȡPROM
	return ret;
}


/*MS5611��ѹADCΪ24λ
 *�õ���������¶�ֵ
 *
 *�¶Ȳ������㹫ʽ��
 *dT = D2 - TREF = D2 - C5 * 2^8
 *TEMP = 20��C + dT * TEMPSENS = 2000 + dT * C6 / 2^23
 */
uint32_t D2 = 0;
int32_t Dt = 0;
void MS5611_Get_Temp(int32_t* Temp)
{
	D2 = MS5611_Get_Conversion(MS5611_D2_OSR_4096);
	
//	Dt = D2 - PROM_C[5]*0x100;
	Dt = D2 - (((uint32_t)PROM_C[5])<<8);
	*Temp  = 2000 + ((Dt*PROM_C[6]) >> 23);
}




/*��ѹ�������㹫ʽ��
 *OFF = OFFT1 + TCO * dT = C2 * 2^16 + (C4 * dT ) / 2^7
 *SENS = SENST1 + TCS * dT = C1 * 2^15 + (C3 * dT ) / 2^8
 *P = D1 * SENS - OFF = (D1 * SENS / 2^21 - OFF) / 2^15
 */
uint32_t D1 = 0;
int64_t OFF = 0;
int64_t SENS = 0;

/*�����¶Ȳ���
 *
if(TEMP<20��C)
{
	T2 = dT^2 / 2^31;
	OFF2 = 5 *(TEMP �C 2000)^2  / 2^1;
	SENS2 = 5 * (TEMP �C 2000)^2 / 2^2;
}
if(TEMP<-15��C)
{
	OFF2 = OFF2 + 7 * (TEMP + 1500)^2;
	SENS2 = SENS2 + 11 * (TEMP + 1500)^2 / 2^1;
}
 
 */
uint64_t T2=0;
uint32_t OFF2=0;
uint32_t SENS2=0;

/*MS5611��ѹADCΪ24λ
 *�õ��������ѹ��ֵ
 *�ֱ��ʣ�0.01mbar
 */
void MS5611_Get_Pressure(int32_t* P,int32_t* Temp)
{
	int64_t Aux1 = 0,Aux2 = 0;
	D1 = MS5611_Get_Conversion(MS5611_D1_OSR_4096);
	OFF = (PROM_C[2]<<16) + ((PROM_C[4] * Dt )>>7);
	SENS = (PROM_C[1]<<15) + ((PROM_C[3] * Dt )>>8);
	
	
	if(*Temp < 2000)//�����¶Ȳ���
	{
		Aux1 = (*Temp - 2000) * (*Temp - 2000);
		T2 = (Dt*Dt)>>31; 
		OFF2 = 2.5 * Aux1;
		SENS2 = 1.25 * Aux1;
		
		if(*Temp < -1500)
		{
			Aux2 = (*Temp + 1500) * (*Temp + 1500);
			OFF2 = OFF2 + 7 * Aux2;
			SENS2 = SENS2 + 5.5 * Aux2;
		}
		
		*Temp = *Temp - T2;
		OFF = OFF - OFF2;
		SENS = SENS - SENS2;	
	}
	*P = (((D1 * SENS)>>21) - OFF) >> 15;
}


/*�õ����θ߶�
 *P����ѹֵ
 *Temp���¶�ֵ
 *���أ�����ֵ
 */
float MS5611_Get_High(int32_t* P,int32_t* Temp)
{
	float hight = 0;
	return hight;
}








