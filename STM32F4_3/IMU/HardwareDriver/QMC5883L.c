#include "QMC5883L.h"

_MAG _Mag = {0};

/*дһ���ֽ�
 *reg���Ĵ�����ַ
 *data������
 *����ֵ:0,����
 *  	����,�������
 */
static unsigned char IIC_Write_One_Byte(unsigned char reg,unsigned char data)
{				   	  	    																 
	QMC_IIC_Start();  
	QMC_IIC_Send_Byte(QMC_ADDR<<1 | 0);//����������ַ+д����                                                    //��������д���� 	 
	if(QMC_IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		QMC_IIC_Stop();		 
		return 1;		
	}	   
	QMC_IIC_Send_Byte(reg);  //д�Ĵ�����ַ                                                      //���͵͵�ַ
	QMC_IIC_Wait_Ack(); 	 //�ȴ�Ӧ�� 										  		   
	QMC_IIC_Send_Byte(data); //��������                                                      //�����ֽ�							   
	if(QMC_IIC_Wait_Ack())	 //�ȴ�Ӧ��
	{
		QMC_IIC_Stop();		 
		return 1;		
	}	 		    	   
	QMC_IIC_Stop();		
	return 0;
}

/*��һ���ֽ�
 *reg���Ĵ�����ַ
 *���ؼĴ�����ֵ
 */
static unsigned char IIC_Read_One_Byte(unsigned char reg)
{
	unsigned char ret = 0;
	QMC_IIC_Start();
	QMC_IIC_Send_Byte(QMC_ADDR<<1 | 0);//����������ַ+д����                                                    //��������д���� 	 
	QMC_IIC_Wait_Ack();//�ȴ�Ӧ��
  	QMC_IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    QMC_IIC_Wait_Ack();		//�ȴ�Ӧ��
	
	QMC_IIC_Start();
	QMC_IIC_Send_Byte(QMC_ADDR<<1 | 1);//����������ַ+������    
	QMC_IIC_Wait_Ack();
	ret=QMC_IIC_Read_Byte(0);//��ȡ����,����nACK 	
	QMC_IIC_Stop();	
	
	return ret;
}


///*IIC����д
// *addr:������ַ 
// *reg:�Ĵ�����ַ
// *len:д�볤��
// *buf:������
// *����ֵ:true,����
//    ����,�������
// */
//static bool QMC_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
//{
//	u8 i; 
//    QMC_IIC_Start(); 
//	QMC_IIC_Send_Byte((addr<<1)|0);//����������ַ+д����	
//	if(QMC_IIC_Wait_Ack())	//�ȴ�Ӧ��
//	{
//		QMC_IIC_Stop();		 
//		return false;		
//	}
//    QMC_IIC_Send_Byte(reg);	//д�Ĵ�����ַ
//    QMC_IIC_Wait_Ack();		//�ȴ�Ӧ��
//	for(i=0;i<len;i++)
//	{
//		QMC_IIC_Send_Byte(buf[i]);	//��������
//		if(QMC_IIC_Wait_Ack())		//�ȴ�ACK
//		{
//			QMC_IIC_Stop();	 
//			return false;		 
//		}		
//	}    
//    QMC_IIC_Stop();	 
//	return true;	
//} 

/*IIC������
 *addr:������ַ
 *reg:Ҫ��ȡ�ļĴ�����ַ
 *len:Ҫ��ȡ�ĳ���
 *buf:��ȡ�������ݴ洢��
 *����ֵ:true,����
 *    ����,�������
 */
static bool QMC_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
 	QMC_IIC_Start(); 
	QMC_IIC_Send_Byte((addr<<1)|0);//����������ַ+д����	
	if(QMC_IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		QMC_IIC_Stop();		 
		return false;		
	}
    QMC_IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    QMC_IIC_Wait_Ack();		//�ȴ�Ӧ��
    QMC_IIC_Start();
	QMC_IIC_Send_Byte((addr<<1)|1);//����������ַ+������	
    QMC_IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	while(len)
	{
		if(len==1)*buf=QMC_IIC_Read_Byte(0);//������,����nACK 
		else *buf=QMC_IIC_Read_Byte(1);		//������,����ACK  
		len--;
		buf++; 
	}    
    QMC_IIC_Stop();	//����һ��ֹͣ���� 
	return true;	
}

/* ��ʼ��QMC5883L
 * ����true��	��ʼ���ɹ�
 * ����false��	��ʼ��ʧ��
 */
bool QMC_Init(void)
{
	unsigned char ID = 0;
	QMC_IIC_Init();
	delay_ms(1);
	IIC_Write_One_Byte(CTRL_REG_1,0x0D);//[1,0,0,1] ��������100hz
	IIC_Write_One_Byte(REG_11,0x01);//Define Set/Reset period

//	IIC_Write_One_Byte(CTRL_REG_2,0x01);//����DRDY�����ϲ����ж�
//	IIC_Write_One_Byte(CTRL_REG_2,0x80);//��λ

	ID = IIC_Read_One_Byte(CTRL_REG_1);//��ȡ����ID
//#if PRINTF
//	printf("QMC5883L_ID =  0x%x\r\n",ID);
//#endif
	if(ID != QMC_ADDR)
	{
		return false;
	}
	return true;
}


/*�õ�������ֵ(ԭʼֵ)
 *mx,my,mz:������x,y,z���ԭʼ����(������)
 *����ֵ:true,�ɹ�
 *   ����,�������
 */
static bool MPU_Get_Magnetometer(short *mx,short *my,short *mz)
{
    u8 buf[6],res=false; 
	res=QMC_Read_Len(QMC_ADDR,MAG_XOUT_L,6,buf);
	if(res==true)
	{
		*mx=((u16)buf[1]<<8)|buf[0];  
		*my=((u16)buf[3]<<8)|buf[2];  
		*mz=((u16)buf[5]<<8)|buf[4];
	} 	
    return res;
}


/*�õ��¶�ֵ(ԭʼֵ)
 */
short MPU_Get_Temperature(void)
{
    u8 buf[2]; 
    short raw;
	QMC_Read_Len(QMC_ADDR,TEMP_L,2,buf); 
    raw=((u16)buf[0]<<8)|buf[1];
    return raw;;
}

/*�õ�������ֵ(���������˲����ֵ)
 *mx,my,mz:������x,y,z����˲����ֵ(������)
 *����ֵ:true,�ɹ�
 *   ����,�������
 */
#define  FILL_NUM  10 //�������ڵ����
int16_t x_offest=10,y_offest=175;
double y_gain=0.967;
bool get_mag_raw(_S16xyz* ma)
{
	u8 res = false; 
	int i = 0;
	int32_t temp1=0,temp2=0,temp3=0;
	
	static uint8_t filter_cnt=0;
	
	static short org_x[FILL_NUM] = {0};//ԭʼ����
	static short org_y[FILL_NUM] = {0};
	static short org_z[FILL_NUM] = {0};
	 
	res = MPU_Get_Magnetometer(org_x + filter_cnt,org_y + filter_cnt,org_z + filter_cnt);

	for(i=0;i<FILL_NUM;i++)  //10��ȵĻ����˲�
	{
		temp1 += org_x[i];
		temp2 += org_y[i];
		temp3 += org_z[i];
	}
	
	ma->x = temp1 / FILL_NUM;
	ma->y = temp2 / FILL_NUM;
	ma->z = temp3 / FILL_NUM;
	
	filter_cnt++;
	if(filter_cnt==FILL_NUM)	filter_cnt=0;
	return res;
}





