#ifndef __FLASH_H__
#define __FLASH_H__

#include "stm32f4xx.h"
#include "stdarg.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"


//FLASH存储数据结构体
typedef struct
{
    char* str;
    int   d;
} ST_FLASH_DATA;


#define FLASH_DEFAULT_SAVE_ADDR ADDR_FLASH_SECTOR_5     //推荐FLASH起始地址

//FLASH 扇区的起始地址
#define ADDR_FLASH_SECTOR_0 ((u32)0x08000000) 	//扇区0起始地址，16 Kbytes  
#define ADDR_FLASH_SECTOR_1 ((u32)0x08004000) 	//扇区1起始地址，16 Kbytes  
#define ADDR_FLASH_SECTOR_2 ((u32)0x08008000) 	//扇区2起始地址，16 Kbytes  
#define ADDR_FLASH_SECTOR_3 ((u32)0x0800C000) 	//扇区3起始地址，16 Kbytes  
#define ADDR_FLASH_SECTOR_4 ((u32)0x08010000) 	//扇区4起始地址，64 Kbytes  
#define ADDR_FLASH_SECTOR_5 ((u32)0x08020000) 	//扇区5起始地址，128 Kbytes  
#define ADDR_FLASH_SECTOR_6 ((u32)0x08040000) 	//扇区6起始地址，128 Kbytes  
#define ADDR_FLASH_SECTOR_7 ((u32)0x08060000) 	//扇区7起始地址，128 Kbytes  
#define ADDR_FLASH_SECTOR_8 ((u32)0x08080000) 	//扇区8起始地址，128 Kbytes  
#define ADDR_FLASH_SECTOR_9 ((u32)0x080A0000) 	//扇区9起始地址，128 Kbytes  
#define ADDR_FLASH_SECTOR_10 ((u32)0x080C0000) 	//扇区10起始地址，128 Kbytes  
#define ADDR_FLASH_SECTOR_11 ((u32)0x080E0000) 	//扇区11起始地址，128 Kbytes  

void FLASH_Read(u32 ReadAddr,                   //读起始地址
                char pBuffer[],                 //数据指针
                u8 Len);                        //数据字节数

void FLASH_Write(u32 WriteAddr,                 //写起始地址，必须为4的倍数
                 char pBuffer[],                //数据指针
                 u8 Len);                       //数据字节数

u8 FLASH_printf(u32 WriteAddr,                  //写起始地址，必须为4的倍数
                const char* fmt,              //数据格式字符串
                ...);                         //其他参数

void FLASH_Save(void);                          //保存FLASH数据

void FLASH_Reload(void);                        //重载保存在FLASH的数据

#endif
