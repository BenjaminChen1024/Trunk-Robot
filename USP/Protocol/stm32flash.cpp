/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    stm32flash.c
  * @author  chenpeiqi
  * @brief   Code for CAN driver in STM32 series MCU, supported packaged:
  *          - STM32Cube_FW_F4_V1.24.0.
  *          - STM32Cube_FW_F1_V1.8.0.
  *          - STM32Cube_FW_H7_V1.5.0.
  * @date    2019-06-12
  * @version 1.1
  * @par Change Log：
  * <table>
  * <tr><th>Date        <th>Version  <th>Author    		    <th>Description
  * <tr><td>2019-06-12  <td> 1.0     <td>Chen Peiqi       <td>Creator
 
  * </table>
  *
  ==============================================================================
                            How to use this driver  
  ==============================================================================
  @note
	-# 此文件没有提供flash的读出操作，因为flash的读出可以直接通过memcpy函数直接读出
  -# ex：memcpy(uint8_t *pBuffer,(uint32_t*)ReadAddr,NumToRead);
  
  ******************************************************************************
  * @attention
  * 
  * if you had modified this file, please make sure your code does not have any 
  * bugs, update the version Number, write dowm your name and the date. The most
  * important thing is make sure the users will have clear and definite under-
  * standing through your new brief.
  *
  * <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
  * All rights reserved.</center></h2>
  ******************************************************************************
*/


#include "stm32flash.h"


/** 
* @brief  对扇区进行擦除  
  * @note 建议在通信前单独使用清空板
          再下载程序，而且写扇区数时
          尽量往中间或者后面的扇区写
* @param   扇区的序号
* @retval 擦除状态
* @author 陈佩奇
*/
uint8_t Erase(uint8_t sector_num)
{
	FLASH_EraseInitTypeDef Flash_Init;
	HAL_StatusTypeDef FlashStatus=HAL_OK;
	uint32_t SectorError=0;

	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | \
	FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    Flash_Init.TypeErase=FLASH_TYPEERASE_SECTORS;
	Flash_Init.Sector=sector_num;
    Flash_Init.NbSectors=1;
	Flash_Init.VoltageRange=FLASH_VOLTAGE_RANGE_3;

	if(HAL_FLASHEx_Erase(&Flash_Init, &SectorError)!=HAL_OK) return ERASE_ERROR ;

	FlashStatus=FLASH_WaitForLastOperation(FLASH_WAITTIME);//等待擦除完成

	if(FlashStatus==HAL_OK)
	{
		HAL_FLASH_Lock();
		return ERASE_OK;
	}
	else return ERASE_ERROR;

}
/**
* @brief  写入
*  @note   flash地址的操作不能用memcpy
 * @param  写入的地址，数据包，写入的字节数
 * @retval 写入状态
 * @author 陈佩奇
 */
 uint8_t Write_Message(uint32_t WriteAddr,uint16_t* pBuffer,uint8_t NumToWrite)
{
	uint32_t endaddr=0;

	endaddr=WriteAddr+NumToWrite;//写入结束地址

	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | \
		FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

	while(WriteAddr<endaddr)
	{
		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,WriteAddr,*pBuffer)!=HAL_OK)
		{
			return FLASH_WRITE_ERROR ;
		}
		WriteAddr+=2;
		pBuffer++;
	}
	HAL_FLASH_Lock();
	return FLASH_WRITE_OK;

}

