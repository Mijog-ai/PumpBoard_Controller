#ifndef __FLASH_H__
#define __FLASH_H__

#include "stm32f4xx.h"
//#define SECTOR_SIZE             (0x4000)	   		

//FLASH起始地址

//#define PARAMETER_MAX_COUNT     100
#define STM32_FLASH_BASE         0x08000000 	//STM32 FLASH的起始地址

#define APP_START_OFFSET_ADDR	 0x00020000

#define STM32_FLASH_EEPROM_BASE  0x08060000 	//STM32 flash存储参数的地址
#define STM32_FLASH_MAIN_BASE    0x08020000 	//STM32 flash存储主程序区域的地址
#define STM32_FLASH_BACKUP_BASE  0x08040000 	//STM32 flash备份区域的地址
#define STM32_FLASH_IAPCTRL_BASE 0x08080000


#define APPLICATION_VALID_FLAG_ADDRESS              (0x0803C000)

#define APPLICATION_VALID_FLAG_VALUE				(0xAC3553CA)
#define REPROGRAMMING_VALID_FLAG_VALUE				(0x5072)


//FLASH 扇区的起始地址
#define ADDR_FLASH_SECTOR_0     ((u32)0x08000000) 	//扇区0起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_1     ((u32)0x08004000) 	//扇区1起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_2     ((u32)0x08008000) 	//扇区2起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_3     ((u32)0x0800C000) 	//扇区3起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_4     ((u32)0x08010000) 	//扇区4起始地址, 64 Kbytes  
#define ADDR_FLASH_SECTOR_5     ((u32)0x08020000) 	//扇区5起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_6     ((u32)0x08040000) 	//扇区6起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_7     ((u32)0x08060000) 	//扇区7起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_8     ((u32)0x08080000) 	//扇区8起始地址, 128 Kbytes  



s32 IAP_EraseSWFlash(u32, u32);


void Flash_Read(u32, u8* Readbuff, u32);
void STMFLASH_Write_byteword(u32, u8 *pBuf, u8);
void STMFLASH_Write_Word(u32, s32 *pBuf, u32);
u32 STMFLASH_ReadWord(u32 faddr);		  	//读出字  
void STMFLASH_Write(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite);		//从指定地址开始写入指定长度的数据
void STMFLASH_Read(u32 ReadAddr,u32 *pBuffer,u32 NumToRead);   		//从指定地址开始读出指定长度的数据
void Save_Parameter_Process(void);	
void InitUserParaProcess(void);
void UpdateSetCmdFunc(void);
FLASH_Status ota_flash_erase(u32 addrx);
FLASH_Status ota_write_byteword(u32 WriteAddr,u8 *pBuffer,u32 NumToWrite);
#endif

