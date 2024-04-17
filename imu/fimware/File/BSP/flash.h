#ifndef __FLASH_H__
#define __FLASH_H__

#include "stm32f4xx.h"
#include "stdarg.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"


//FLASH�洢���ݽṹ��
typedef struct
{
    char* str;
    int   d;
} ST_FLASH_DATA;


#define FLASH_DEFAULT_SAVE_ADDR ADDR_FLASH_SECTOR_5     //�Ƽ�FLASH��ʼ��ַ

//FLASH ��������ʼ��ַ
#define ADDR_FLASH_SECTOR_0 ((u32)0x08000000) 	//����0��ʼ��ַ��16 Kbytes  
#define ADDR_FLASH_SECTOR_1 ((u32)0x08004000) 	//����1��ʼ��ַ��16 Kbytes  
#define ADDR_FLASH_SECTOR_2 ((u32)0x08008000) 	//����2��ʼ��ַ��16 Kbytes  
#define ADDR_FLASH_SECTOR_3 ((u32)0x0800C000) 	//����3��ʼ��ַ��16 Kbytes  
#define ADDR_FLASH_SECTOR_4 ((u32)0x08010000) 	//����4��ʼ��ַ��64 Kbytes  
#define ADDR_FLASH_SECTOR_5 ((u32)0x08020000) 	//����5��ʼ��ַ��128 Kbytes  
#define ADDR_FLASH_SECTOR_6 ((u32)0x08040000) 	//����6��ʼ��ַ��128 Kbytes  
#define ADDR_FLASH_SECTOR_7 ((u32)0x08060000) 	//����7��ʼ��ַ��128 Kbytes  
#define ADDR_FLASH_SECTOR_8 ((u32)0x08080000) 	//����8��ʼ��ַ��128 Kbytes  
#define ADDR_FLASH_SECTOR_9 ((u32)0x080A0000) 	//����9��ʼ��ַ��128 Kbytes  
#define ADDR_FLASH_SECTOR_10 ((u32)0x080C0000) 	//����10��ʼ��ַ��128 Kbytes  
#define ADDR_FLASH_SECTOR_11 ((u32)0x080E0000) 	//����11��ʼ��ַ��128 Kbytes  

void FLASH_Read(u32 ReadAddr,                   //����ʼ��ַ
                char pBuffer[],                 //����ָ��
                u8 Len);                        //�����ֽ���

void FLASH_Write(u32 WriteAddr,                 //д��ʼ��ַ������Ϊ4�ı���
                 char pBuffer[],                //����ָ��
                 u8 Len);                       //�����ֽ���

u8 FLASH_printf(u32 WriteAddr,                  //д��ʼ��ַ������Ϊ4�ı���
                const char* fmt,              //���ݸ�ʽ�ַ���
                ...);                         //��������

void FLASH_Save(void);                          //����FLASH����

void FLASH_Reload(void);                        //���ر�����FLASH������

#endif
