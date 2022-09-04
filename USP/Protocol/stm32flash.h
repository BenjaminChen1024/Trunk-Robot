/*为了使用起来方便
 * 不直接对PDO的通信参数变量赋予固定地址
 * 而是直接等初始化完成后直接拷贝
 * */


#ifndef _STM32FLASH_H
#define _STM32FLASH_H

/*include---------------------------------------------------------------*/

#include "main.h"
#include "string.h"
/*define---------------------------------------------------------------*/

/*falsh主存储器sector的首地址，f405RG共12个扇区*/
#define ADDR_FLASH_SECTOR_0   0x08000000
#define ADDR_FLASH_SECTOR_1   0x08004000
#define ADDR_FLASH_SECTOR_2   0x08008000
#define ADDR_FLASH_SECTOR_3   0x0800C000
#define ADDR_FLASH_SECTOR_4   0x08010000
#define ADDR_FLASH_SECTOR_5   0x08020000
#define ADDR_FLASH_SECTOR_6   0x08040000
#define ADDR_FLASH_SECTOR_7   0x08060000
#define ADDR_FLASH_SECTOR_8   0x08080000
#define ADDR_FLASH_SECTOR_9   0x080A0000
#define ADDR_FLASH_SECTOR_10  0x080C0000
#define ADDR_FLASH_SECTOR_11  0x080E0000


/*等待时间，在寄存器擦除过程中不可进行读取和写入*/
#define FLASH_WAITTIME 10

/*flash擦除状态预定义*/
#define ERASE_ERROR    0
#define ERASE_OK       1

/*flash写入状态定义*/
#define FLASH_WRITE_ERROR   0
#define FLASH_WRITE_OK      1

/*C++---------------------------------------------------------------*/
#ifdef __cplusplus
 
#endif
/*C---------------------------------------------------------------*/
#ifdef __cplusplus
 extern "C" {
#endif
 /*flash读写操作*/
 uint8_t Erase(uint8_t sector_num);
 uint8_t Write_Message(uint32_t WriteAddr,uint16_t* pBuffer,uint8_t NumToWrite);
 void Read_Message(uint32_t ReadAddr,uint8_t *pBuffer,uint8_t NumToRead);
#ifdef __cplusplus
}
#endif
 /*---------------------------------------------------------------*/
#endif
