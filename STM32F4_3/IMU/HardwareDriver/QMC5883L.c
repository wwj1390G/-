#include "QMC5883L.h"

_MAG _Mag = {0};

/*写一个字节
 *reg：寄存器地址
 *data：数据
 *返回值:0,正常
 *  	其他,错误代码
 */
static unsigned char IIC_Write_One_Byte(unsigned char reg,unsigned char data)
{				   	  	    																 
	QMC_IIC_Start();  
	QMC_IIC_Send_Byte(QMC_ADDR<<1 | 0);//发送器件地址+写命令                                                    //发送器件写命令 	 
	if(QMC_IIC_Wait_Ack())	//等待应答
	{
		QMC_IIC_Stop();		 
		return 1;		
	}	   
	QMC_IIC_Send_Byte(reg);  //写寄存器地址                                                      //发送低地址
	QMC_IIC_Wait_Ack(); 	 //等待应答 										  		   
	QMC_IIC_Send_Byte(data); //发送数据                                                      //发送字节							   
	if(QMC_IIC_Wait_Ack())	 //等待应答
	{
		QMC_IIC_Stop();		 
		return 1;		
	}	 		    	   
	QMC_IIC_Stop();		
	return 0;
}

/*读一个字节
 *reg：寄存器地址
 *返回寄存器的值
 */
static unsigned char IIC_Read_One_Byte(unsigned char reg)
{
	unsigned char ret = 0;
	QMC_IIC_Start();
	QMC_IIC_Send_Byte(QMC_ADDR<<1 | 0);//发送器件地址+写命令                                                    //发送器件写命令 	 
	QMC_IIC_Wait_Ack();//等待应答
  	QMC_IIC_Send_Byte(reg);	//写寄存器地址
    QMC_IIC_Wait_Ack();		//等待应答
	
	QMC_IIC_Start();
	QMC_IIC_Send_Byte(QMC_ADDR<<1 | 1);//发送器件地址+读命令    
	QMC_IIC_Wait_Ack();
	ret=QMC_IIC_Read_Byte(0);//读取数据,发送nACK 	
	QMC_IIC_Stop();	
	
	return ret;
}


///*IIC连续写
// *addr:器件地址 
// *reg:寄存器地址
// *len:写入长度
// *buf:数据区
// *返回值:true,正常
//    其他,错误代码
// */
//static bool QMC_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
//{
//	u8 i; 
//    QMC_IIC_Start(); 
//	QMC_IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
//	if(QMC_IIC_Wait_Ack())	//等待应答
//	{
//		QMC_IIC_Stop();		 
//		return false;		
//	}
//    QMC_IIC_Send_Byte(reg);	//写寄存器地址
//    QMC_IIC_Wait_Ack();		//等待应答
//	for(i=0;i<len;i++)
//	{
//		QMC_IIC_Send_Byte(buf[i]);	//发送数据
//		if(QMC_IIC_Wait_Ack())		//等待ACK
//		{
//			QMC_IIC_Stop();	 
//			return false;		 
//		}		
//	}    
//    QMC_IIC_Stop();	 
//	return true;	
//} 

/*IIC连续读
 *addr:器件地址
 *reg:要读取的寄存器地址
 *len:要读取的长度
 *buf:读取到的数据存储区
 *返回值:true,正常
 *    其他,错误代码
 */
static bool QMC_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
 	QMC_IIC_Start(); 
	QMC_IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
	if(QMC_IIC_Wait_Ack())	//等待应答
	{
		QMC_IIC_Stop();		 
		return false;		
	}
    QMC_IIC_Send_Byte(reg);	//写寄存器地址
    QMC_IIC_Wait_Ack();		//等待应答
    QMC_IIC_Start();
	QMC_IIC_Send_Byte((addr<<1)|1);//发送器件地址+读命令	
    QMC_IIC_Wait_Ack();		//等待应答 
	while(len)
	{
		if(len==1)*buf=QMC_IIC_Read_Byte(0);//读数据,发送nACK 
		else *buf=QMC_IIC_Read_Byte(1);		//读数据,发送ACK  
		len--;
		buf++; 
	}    
    QMC_IIC_Stop();	//产生一个停止条件 
	return true;	
}

/* 初始化QMC5883L
 * 返回true：	初始化成功
 * 返回false：	初始化失败
 */
bool QMC_Init(void)
{
	unsigned char ID = 0;
	QMC_IIC_Init();
	delay_ms(1);
	IIC_Write_One_Byte(CTRL_REG_1,0x0D);//[1,0,0,1] 连续读，100hz
	IIC_Write_One_Byte(REG_11,0x01);//Define Set/Reset period

//	IIC_Write_One_Byte(CTRL_REG_2,0x01);//禁用DRDY引脚上产生中断
//	IIC_Write_One_Byte(CTRL_REG_2,0x80);//复位

	ID = IIC_Read_One_Byte(CTRL_REG_1);//读取器件ID
//#if PRINTF
//	printf("QMC5883L_ID =  0x%x\r\n",ID);
//#endif
	if(ID != QMC_ADDR)
	{
		return false;
	}
	return true;
}


/*得到磁力计值(原始值)
 *mx,my,mz:磁力计x,y,z轴的原始读数(带符号)
 *返回值:true,成功
 *   其他,错误代码
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


/*得到温度值(原始值)
 */
short MPU_Get_Temperature(void)
{
    u8 buf[2]; 
    short raw;
	QMC_Read_Len(QMC_ADDR,TEMP_L,2,buf); 
    raw=((u16)buf[0]<<8)|buf[1];
    return raw;;
}

/*得到磁力计值(滑动窗口滤波后的值)
 *mx,my,mz:磁力计x,y,z轴的滤波后的值(带符号)
 *返回值:true,成功
 *   其他,错误代码
 */
#define  FILL_NUM  10 //滑动窗口的深度
int16_t x_offest=10,y_offest=175;
double y_gain=0.967;
bool get_mag_raw(_S16xyz* ma)
{
	u8 res = false; 
	int i = 0;
	int32_t temp1=0,temp2=0,temp3=0;
	
	static uint8_t filter_cnt=0;
	
	static short org_x[FILL_NUM] = {0};//原始数据
	static short org_y[FILL_NUM] = {0};
	static short org_z[FILL_NUM] = {0};
	 
	res = MPU_Get_Magnetometer(org_x + filter_cnt,org_y + filter_cnt,org_z + filter_cnt);

	for(i=0;i<FILL_NUM;i++)  //10深度的滑动滤波
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





