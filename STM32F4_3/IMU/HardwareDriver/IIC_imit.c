#include "IIC_imit.h"


//��ʼ��IIC
void imit_IIC_Init(void)
{		
	static bool imit_IIC = false;//��ֹ���γ�ʼ��	
	if(imit_IIC == false)
	{
		imit_IIC = true;
		GPIO_InitTypeDef GPIO_InitStructure;

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��GPIOBʱ��

		//GPIO��ʼ������
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
		GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��
		IIC_SCL=1;
		IIC_SDA=1;
	}
	
}
//����IIC��ʼ�ź�
void imit_IIC_Start(void)
{
	SDA_OUT();     //sda�����
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	delay_us(4);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void imit_IIC_Stop(void)
{
	SDA_OUT();//sda�����
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL=1; 
	IIC_SDA=1;//����I2C���߽����ź�
	delay_us(4);							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 imit_IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����  
	IIC_SDA=1;delay_us(1);	   
	IIC_SCL=1;delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			QMC_IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
void imit_IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}
//������ACKӦ��		    
void imit_IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void imit_IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);   //��TEA5767��������ʱ���Ǳ����
		IIC_SCL=1;
		delay_us(2); 
		IIC_SCL=0;	
		delay_us(2);
    }	 
} 	    
/*��1���ֽ�
 *ack = 1������ACK
 *ack = 0������nACK   
 */
u8 imit_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        delay_us(2);
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		delay_us(1); 
    }					 
    if (ack)
		QMC_IIC_Ack(); //����ACK   
    else
        QMC_IIC_NAck();//����nACK
    return receive;
}



