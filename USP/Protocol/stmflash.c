#include "stmflash.h"

//��ȡָ����ַ����(32λ����) 
//faddr:����ַ 
//����ֵ:��Ӧ����.
uint32_t STMFLASH_ReadWord(uint32_t faddr)
{
	return *(__IO uint32_t *)faddr; 
}

//��ȡĳ����ַ���ڵ�flash����
//addr:flash��ַ
//����ֵ:0~11,��addr���ڵ�����
uint8_t STMFLASH_GetFlashSector(uint32_t addr)
{
	if(addr<ADDR_FLASH_SECTOR_1)return FLASH_SECTOR_0;
	else if(addr<ADDR_FLASH_SECTOR_2)return FLASH_SECTOR_1;
	else if(addr<ADDR_FLASH_SECTOR_3)return FLASH_SECTOR_2;
	else if(addr<ADDR_FLASH_SECTOR_4)return FLASH_SECTOR_3;
	else if(addr<ADDR_FLASH_SECTOR_5)return FLASH_SECTOR_4;
	else if(addr<ADDR_FLASH_SECTOR_6)return FLASH_SECTOR_5;
	else if(addr<ADDR_FLASH_SECTOR_7)return FLASH_SECTOR_6;
	else if(addr<ADDR_FLASH_SECTOR_8)return FLASH_SECTOR_7;
	else if(addr<ADDR_FLASH_SECTOR_9)return FLASH_SECTOR_8;
	else if(addr<ADDR_FLASH_SECTOR_10)return FLASH_SECTOR_9;
	else if(addr<ADDR_FLASH_SECTOR_11)return FLASH_SECTOR_10; 
	else if(addr<ADDR_FLASH_SECTOR_12)return FLASH_SECTOR_11; 

}

//��ָ����ַ��ʼд��ָ�����ȵ�����
//�ر�ע��:��ΪSTM32F4������ʵ��̫��,û�취���ر�����������,���Ա�����
//         д��ַ�����0XFF,��ô���Ȳ������������Ҳ�������������.����
//         д��0XFF�ĵ�ַ,�����������������ݶ�ʧ.����д֮ǰȷ��������
//         û����Ҫ����,��������������Ȳ�����,Ȼ����������д. 
//�ú�����OTP����Ҳ��Ч!��������дOTP��!
//OTP�����ַ��Χ:0X1FFF7800~0X1FFF7A0F(ע�⣺���16�ֽڣ�����OTP���ݿ�����������д����)
//WriteAddr:��ʼ��ַ(�˵�ַ����Ϊ4�ı���!!)
//pBuffer:����ָ��
//NumToWrite:��(32λ)��(����Ҫд���32λ���ݵĸ���.) 
void STMFLASH_Write(uint32_t WriteAddr,uint32_t *pBuffer,uint32_t NumToWrite)	
{
	FLASH_EraseInitTypeDef FlashEraseInit;
  HAL_StatusTypeDef FlashStatus=HAL_OK;
  uint32_t SectorError=0;
	uint32_t addrx=0;
	uint32_t endaddr=0;	
  if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)	return;	//�Ƿ���ַ
 
	HAL_FLASH_Unlock();             //����	
	addrx=WriteAddr;								//д�����ʼ��ַ
	endaddr=WriteAddr+NumToWrite*4;	//д��Ľ�����ַ
    
	if(addrx<0X1FFF0000)
	{
		while(addrx<endaddr)		//ɨ��һ���ϰ�.(�Է�FFFFFFFF�ĵط�,�Ȳ���)
		{
			if(STMFLASH_ReadWord(addrx)!=0XFFFFFFFF)//�з�0XFFFFFFFF�ĵط�,Ҫ�����������
			{   
				FlashEraseInit.TypeErase=FLASH_TYPEERASE_SECTORS;       //�������ͣ��������� 
				FlashEraseInit.Sector=STMFLASH_GetFlashSector(addrx);   //Ҫ����������
				FlashEraseInit.NbSectors=1;                             //һ��ֻ����һ������
				FlashEraseInit.VoltageRange=FLASH_VOLTAGE_RANGE_3;      //��ѹ��Χ��VCC=2.7~3.6V֮��!!
				if(HAL_FLASHEx_Erase(&FlashEraseInit,&SectorError)!=HAL_OK) break;//����������	

			}
			else 
			{
				addrx+=4;
			}
			FLASH_WaitForLastOperation(FLASH_WAITETIME);               //�ȴ��ϴβ������
		}
	}
  FlashStatus=FLASH_WaitForLastOperation(FLASH_WAITETIME);       //�ȴ��ϴβ������
	if(FlashStatus==HAL_OK)
	{
		while(WriteAddr<endaddr)//д����
		{
            if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,WriteAddr,*pBuffer)!=HAL_OK)//д������
			{ 
				break;	//д���쳣
			}
			WriteAddr+=4;
			pBuffer++;
		} 
	}
	HAL_FLASH_Lock();           //����
}
		
		
 

//��ָ����ַ��ʼ����ָ�����ȵ�����
//ReadAddr:��ʼ��ַ
//pBuffer:����ָ��
//NumToRead:��(32λ)��
void STMFLASH_Read(uint32_t ReadAddr,uint32_t *pBuffer,uint32_t NumToRead)   	
{
	uint32_t i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);//��ȡ4���ֽ�.
		ReadAddr+=4;//ƫ��4���ֽ�.	
	}
}

/*-----------------------------------������-------------------------------------------------*/
/*
*@note:flashд����
*@param:Mineral_Num
				LeftLift_Encoder
				RightLift_Encoder
*/
void Flash_Write(uint32_t Mineral_Num,uint32_t LeftLift_Encoder,uint32_t RightLift_Encoder)   	
{
	static uint32_t temp_data[3];
	temp_data[0] = Mineral_Num;
	temp_data[1] = LeftLift_Encoder;
	temp_data[2] = RightLift_Encoder;
	STMFLASH_Write(0x080E0000,temp_data,3);//д��һ���� 
}

/*
*@note:flash������
*@param: 3λuint32_t�Ľ������ݵ�����
*/
void Flash_Read(uint32_t *array)
{
	STMFLASH_Read(0x080E0000,array,3);
}
