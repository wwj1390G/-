#include "flash.h"
#include "calm_acc.h"
#include "calm_gyro.h"
#include "calm_mag.h"

#include "my_math.h"
/*************************************************************
 * 内存分配：float型数据
 * 0->5		acc校准数据
 * 6->8 		gyro校准数据
 * 9->14		mag校准数据
 * 
 * 15->15+180	PID数据
*************************************************************/
 
 
//读取指定地址的一个字(32位数据) 
//faddr:读地址 
//返回值:对应数据.
u32 STMFLASH_ReadWord(u32 faddr)
{
	return *(vu32*)faddr; 
}  

u16 STMFLASH_ReadHalfWord(u32 faddr)
{
	return *(vu16*)faddr; 
}  

u8 STMFLASH_ReadByte(u32 faddr)
{
	return *(vu8*)faddr; 
}  
//获取某个地址所在的flash扇区
//addr:flash地址
//返回值:0~11,即addr所在的扇区
uint16_t STMFLASH_GetFlashSector(u32 addr)
{
	if(addr<ADDR_FLASH_SECTOR_1)return FLASH_Sector_0;
	else if(addr<ADDR_FLASH_SECTOR_2)return FLASH_Sector_1;
	else if(addr<ADDR_FLASH_SECTOR_3)return FLASH_Sector_2;
	else if(addr<ADDR_FLASH_SECTOR_4)return FLASH_Sector_3;
	else if(addr<ADDR_FLASH_SECTOR_5)return FLASH_Sector_4;
	else if(addr<ADDR_FLASH_SECTOR_6)return FLASH_Sector_5;
	else if(addr<ADDR_FLASH_SECTOR_7)return FLASH_Sector_6;
	else if(addr<ADDR_FLASH_SECTOR_8)return FLASH_Sector_7;
	else if(addr<ADDR_FLASH_SECTOR_9)return FLASH_Sector_8;
	else if(addr<ADDR_FLASH_SECTOR_10)return FLASH_Sector_9;
	else if(addr<ADDR_FLASH_SECTOR_11)return FLASH_Sector_10; 
	return FLASH_Sector_11;	
}
//从指定地址开始写入指定长度的数据
//特别注意:因为STM32F4的扇区实在太大,没办法本地保存扇区数据,所以本函数
//         写地址如果非0XFF,那么会先擦除整个扇区且不保存扇区数据.所以
//         写非0XFF的地址,将导致整个扇区数据丢失.建议写之前确保扇区里
//         没有重要数据,最好是整个扇区先擦除了,然后慢慢往后写. 
//该函数对OTP区域也有效!可以用来写OTP区!
//OTP区域地址范围:0X1FFF7800~0X1FFF7A0F
//WriteAddr:起始地址(此地址必须为4的倍数!!)
//pBuffer:数据指针
//NumToWrite:字(32位)数(就是要写入的32位数据的个数.) 
void STMFLASH_Write(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite)	
{ 
	FLASH_Status status = FLASH_COMPLETE;
	u32 addrx=0;
	u32 endaddr=0;	
	if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)return;	//非法地址
	FLASH_Unlock();									//解锁 
	FLASH_DataCacheCmd(DISABLE);//FLASH擦除期间,必须禁止数据缓存

	addrx=WriteAddr;				//写入的起始地址
	endaddr=WriteAddr+NumToWrite*4;	//写入的结束地址
	if(addrx<0X1FFF0000)			//只有主存储区,才需要执行擦除操作!!
	{
		while(addrx<endaddr)		//扫清一切障碍.(对非FFFFFFFF的地方,先擦除)
		{
			if(STMFLASH_ReadWord(addrx)!=0XFFFFFFFF)//有非0XFFFFFFFF的地方,要擦除这个扇区
			{   
				status=FLASH_EraseSector(STMFLASH_GetFlashSector(addrx),VoltageRange_3);//VCC=2.7~3.6V之间!!
				if(status!=FLASH_COMPLETE)break;	//发生错误了
			}else addrx+=4;
		} 
	}
	if(status==FLASH_COMPLETE)
	{
		while(WriteAddr<endaddr)//写数据
		{
			if(FLASH_ProgramWord(WriteAddr,*pBuffer)!=FLASH_COMPLETE)//写入数据
			{ 
				
				break;	//写入异常
			}
			WriteAddr+=4;
			pBuffer++;
		} 
	}
	FLASH_DataCacheCmd(ENABLE);	//FLASH擦除结束,开启数据缓存
	FLASH_Lock();//上锁
} 

//从指定地址开始读出指定长度的数据
//ReadAddr:起始地址
//pBuffer:数据指针
//NumToRead:字(4位)数
void STMFLASH_Read(u32 ReadAddr,u32 *pBuffer,u32 NumToRead)   	
{
	u32 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);//读取4个字节.
		ReadAddr+=4;//偏移4个字节.	
	}
}


/*************************************************/

/*************************************************/



//ReadAddr:开始读数的地址  
//返回值  :读到的数据
float Flash_ReadOneFloat(u32 ReadAddr)
{
	float Date;
	*(u32 *)&Date = STMFLASH_ReadWord(ReadAddr);
	return Date;
}

//向指定地址写入一个 float型 数据
//WriteAddr  :写入数据的目的地址    
//DataToWrite:要写入的数据
void Flash_WriteOneFloat(u32 WriteAddr,float DataToWrite)
{
	STMFLASH_Write(WriteAddr,(u32*)&DataToWrite,1);	
}


void Flash_Write_acc(void)
{
	float buf[15] ={0};
	buf[0] = _acc_calm.offset_f.x;
	buf[1] = _acc_calm.offset_f.y;
	buf[2] = _acc_calm.offset_f.z;
	buf[3] = _acc_calm.scale_f.x;
	buf[4] = _acc_calm.scale_f.y;
	buf[5] = _acc_calm.scale_f.z;
	
	for(int i=6;i<15;i++)
	{
		buf[i] = Flash_ReadOneFloat(FLASH_SAVE_ADDR+4*i);
	}
	
	STMFLASH_Write(FLASH_SAVE_ADDR,(u32*)&buf,15);
	
	
	
	
	//调试打印
#if PRINTF
	printf("\r\n\r\n");
	printf("flash write acc ");
	for(int i=0;i<12;i++)
	{
		printf("%0.2f \t",Flash_ReadOneFloat(FLASH_SAVE_ADDR+4*i));
	}
	printf("\r\n");
#endif
}


void Flash_Write_gyro(void)
{
	float buf[15] ={0};
	
	for(int i=0;i<6;i++)
	{
		buf[i] = Flash_ReadOneFloat(FLASH_SAVE_ADDR+4*i);
	}
	buf[6] = _gyro_calm.offset_f.x;
	buf[7] = _gyro_calm.offset_f.y;
	buf[8] = _gyro_calm.offset_f.z;
	
	for(int i=9;i<15;i++)
	{
		buf[i] = Flash_ReadOneFloat(FLASH_SAVE_ADDR+4*i);
	}
	STMFLASH_Write(FLASH_SAVE_ADDR,(u32*)&buf,15);
	
	
	
	
	
	//调试打印
#if PRINTF
	printf("\r\n\r\n");
	printf("flash write gyro ");
	for(int i=0;i<12;i++)
	{
		printf("%0.2f \t",Flash_ReadOneFloat(FLASH_SAVE_ADDR + 4*i));
	}
	printf("\r\n");
#endif
}

void Flash_Write_mag(void)
{
	float buf[15] ={0};
	
	for(int i=0;i<9;i++)
	{
		buf[i] = Flash_ReadOneFloat(FLASH_SAVE_ADDR+4*i);
	}
	
	buf[9]  = _mag_calm.offset.x;
	buf[10] = _mag_calm.offset.y;
	buf[11] = _mag_calm.offset.z;
	
	buf[12] = _mag_calm.x_gain;
	buf[13] = _mag_calm.y_gain;
	buf[14] = _mag_calm.z_gain;
	
	STMFLASH_Write(FLASH_SAVE_ADDR,(u32*)&buf,15);
	
	
	
	//调试打印
#if PRINTF
	printf("\r\n\r\n");
	printf("flash write mag ");
	for(int i=0;i<12;i++)
	{
		printf("%0.2f \t",Flash_ReadOneFloat(FLASH_SAVE_ADDR + 4*i));
	}
	printf("\r\n");
#endif
}


void Flash_Write_PID(void)
{

}



void Flash_Read(void)
{
	float buf[15];
	unsigned char success = 0;
	
	//初始化标度系数和零位误差系数
	for(uint8_t i=0;i<3;i++)
	{
		_acc_calm.K[i] = 1.0f;
		_acc_calm.B[i] = 0.0f;
	}
	
	for(int i=0;i<15;i++)
	{
		buf[i] = Flash_ReadOneFloat(FLASH_SAVE_ADDR+4*i);
	}
	_acc_calm.offset_f.x = buf[0];
	_acc_calm.offset_f.y = buf[1];
	_acc_calm.offset_f.z = buf[2];
	_acc_calm.scale_f.x = buf[3];
	_acc_calm.scale_f.y = buf[4];
	_acc_calm.scale_f.z = buf[5];
	
	_gyro_calm.offset_f.x = buf[6];
	_gyro_calm.offset_f.y = buf[7];
	_gyro_calm.offset_f.z = buf[8];
	
	_mag_calm.offset.x = buf[9];
	_mag_calm.offset.y = buf[10];
	_mag_calm.offset.z = buf[11];
	
	_mag_calm.x_gain = buf[12];
	_mag_calm.y_gain = buf[13];
	_mag_calm.z_gain = buf[14];
	
	// sanity check offsets (3.5 is roughly 3/10th of a G, 5.0 is roughly half a G)
    if(my_abs(_acc_calm.offset_f.x) > 5.0f || my_abs(_acc_calm.offset_f.y) > 5.0f || my_abs(_acc_calm.offset_f.z) > 5.0f)
    {
        success = 1;
    }  
		 // sanity check scale
    if(my_abs(_acc_calm.scale_f.x-1.0f)>0.5f|| my_abs(_acc_calm.scale_f.y-1.0f)>0.5f|| my_abs(_acc_calm.scale_f.z-1.0f)>0.5f)
    {
        success = 2;
    }
	
	
	if(success == 0)    
    {
        //加速度计 偏零
        _acc_calm.B[0] = _acc_calm.offset_f.x;
        _acc_calm.B[1] = _acc_calm.offset_f.y;
        _acc_calm.B[2] = _acc_calm.offset_f.z;
        
        //加速度计 比例因子
        _acc_calm.K[0] = _acc_calm.scale_f.x;
        _acc_calm.K[1] = _acc_calm.scale_f.y;
        _acc_calm.K[2] = _acc_calm.scale_f.z;
    }

	
//	//陀螺仪偏零设置
//	_set_val(&_Mpu.gyro_offset,&_gyro_calm.offset_f);
	
	//磁力计偏零设置
	_set_val(&_Mpu.mag_offset,&_mag_calm.offset);
}





