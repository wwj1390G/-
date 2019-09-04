#include "flash.h"
#include "calm_acc.h"
#include "calm_gyro.h"
#include "calm_mag.h"

#include "my_math.h"
/*************************************************************
 * �ڴ���䣺float������
 * 0->5		accУ׼����
 * 6->8 		gyroУ׼����
 * 9->14		magУ׼����
 * 
 * 15->15+180	PID����
*************************************************************/
 
 
//��ȡָ����ַ��һ����(32λ����) 
//faddr:����ַ 
//����ֵ:��Ӧ����.
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
//��ȡĳ����ַ���ڵ�flash����
//addr:flash��ַ
//����ֵ:0~11,��addr���ڵ�����
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
//��ָ����ַ��ʼд��ָ�����ȵ�����
//�ر�ע��:��ΪSTM32F4������ʵ��̫��,û�취���ر�����������,���Ա�����
//         д��ַ�����0XFF,��ô���Ȳ������������Ҳ�������������.����
//         д��0XFF�ĵ�ַ,�����������������ݶ�ʧ.����д֮ǰȷ��������
//         û����Ҫ����,��������������Ȳ�����,Ȼ����������д. 
//�ú�����OTP����Ҳ��Ч!��������дOTP��!
//OTP�����ַ��Χ:0X1FFF7800~0X1FFF7A0F
//WriteAddr:��ʼ��ַ(�˵�ַ����Ϊ4�ı���!!)
//pBuffer:����ָ��
//NumToWrite:��(32λ)��(����Ҫд���32λ���ݵĸ���.) 
void STMFLASH_Write(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite)	
{ 
	FLASH_Status status = FLASH_COMPLETE;
	u32 addrx=0;
	u32 endaddr=0;	
	if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)return;	//�Ƿ���ַ
	FLASH_Unlock();									//���� 
	FLASH_DataCacheCmd(DISABLE);//FLASH�����ڼ�,�����ֹ���ݻ���

	addrx=WriteAddr;				//д�����ʼ��ַ
	endaddr=WriteAddr+NumToWrite*4;	//д��Ľ�����ַ
	if(addrx<0X1FFF0000)			//ֻ�����洢��,����Ҫִ�в�������!!
	{
		while(addrx<endaddr)		//ɨ��һ���ϰ�.(�Է�FFFFFFFF�ĵط�,�Ȳ���)
		{
			if(STMFLASH_ReadWord(addrx)!=0XFFFFFFFF)//�з�0XFFFFFFFF�ĵط�,Ҫ�����������
			{   
				status=FLASH_EraseSector(STMFLASH_GetFlashSector(addrx),VoltageRange_3);//VCC=2.7~3.6V֮��!!
				if(status!=FLASH_COMPLETE)break;	//����������
			}else addrx+=4;
		} 
	}
	if(status==FLASH_COMPLETE)
	{
		while(WriteAddr<endaddr)//д����
		{
			if(FLASH_ProgramWord(WriteAddr,*pBuffer)!=FLASH_COMPLETE)//д������
			{ 
				
				break;	//д���쳣
			}
			WriteAddr+=4;
			pBuffer++;
		} 
	}
	FLASH_DataCacheCmd(ENABLE);	//FLASH��������,�������ݻ���
	FLASH_Lock();//����
} 

//��ָ����ַ��ʼ����ָ�����ȵ�����
//ReadAddr:��ʼ��ַ
//pBuffer:����ָ��
//NumToRead:��(4λ)��
void STMFLASH_Read(u32 ReadAddr,u32 *pBuffer,u32 NumToRead)   	
{
	u32 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);//��ȡ4���ֽ�.
		ReadAddr+=4;//ƫ��4���ֽ�.	
	}
}


/*************************************************/

/*************************************************/



//ReadAddr:��ʼ�����ĵ�ַ  
//����ֵ  :����������
float Flash_ReadOneFloat(u32 ReadAddr)
{
	float Date;
	*(u32 *)&Date = STMFLASH_ReadWord(ReadAddr);
	return Date;
}

//��ָ����ַд��һ�� float�� ����
//WriteAddr  :д�����ݵ�Ŀ�ĵ�ַ    
//DataToWrite:Ҫд�������
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
	
	
	
	
	//���Դ�ӡ
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
	
	
	
	
	
	//���Դ�ӡ
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
	
	
	
	//���Դ�ӡ
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
	
	//��ʼ�����ϵ������λ���ϵ��
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
        //���ٶȼ� ƫ��
        _acc_calm.B[0] = _acc_calm.offset_f.x;
        _acc_calm.B[1] = _acc_calm.offset_f.y;
        _acc_calm.B[2] = _acc_calm.offset_f.z;
        
        //���ٶȼ� ��������
        _acc_calm.K[0] = _acc_calm.scale_f.x;
        _acc_calm.K[1] = _acc_calm.scale_f.y;
        _acc_calm.K[2] = _acc_calm.scale_f.z;
    }

	
//	//������ƫ������
//	_set_val(&_Mpu.gyro_offset,&_gyro_calm.offset_f);
	
	//������ƫ������
	_set_val(&_Mpu.mag_offset,&_mag_calm.offset);
}





