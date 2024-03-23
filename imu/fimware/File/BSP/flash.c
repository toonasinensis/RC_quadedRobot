#include "flash.h"

ST_FLASH_DATA FLASH_Data[]=
{
    {"YawPosE_Max_history", 0},
    {"YawSpeedE_Max_history", 0},
    {"YawTD.x1_Max_history", 0},
    {"YawTD.x2_Max_history", 0},
};

//从指定地址开始读出指定长度的数据
void FLASH_Read(u32 ReadAddr,               //读起始地址
                char pBuffer[],             //数据指针
                u8 Len)                     //数据字节数
{
    u8* addr = (u8*)ReadAddr;

    for(u32 i=0; i<Len; i++)
    {
        pBuffer[i] = *addr;
        addr++;
    }
}

//获取某个地址所在的flash扇区
//addr:flash地址
//返回值:0~11,即addr所在的扇区
static u16 GetFlashSector(u32 addr)
{
    if(addr<ADDR_FLASH_SECTOR_1) return FLASH_Sector_0;
    else if(addr<ADDR_FLASH_SECTOR_2) return FLASH_Sector_1;
    else if(addr<ADDR_FLASH_SECTOR_3) return FLASH_Sector_2;
    else if(addr<ADDR_FLASH_SECTOR_4) return FLASH_Sector_3;
    else if(addr<ADDR_FLASH_SECTOR_5) return FLASH_Sector_4;
    else if(addr<ADDR_FLASH_SECTOR_6) return FLASH_Sector_5;
    else if(addr<ADDR_FLASH_SECTOR_7) return FLASH_Sector_6;
    else if(addr<ADDR_FLASH_SECTOR_8) return FLASH_Sector_7;
    else if(addr<ADDR_FLASH_SECTOR_9) return FLASH_Sector_8;
    else if(addr<ADDR_FLASH_SECTOR_10) return FLASH_Sector_9;
    else if(addr<ADDR_FLASH_SECTOR_11) return FLASH_Sector_10;
    return FLASH_Sector_11;
}

//从指定地址（可以跨扇区）开始写入指定长度的数据
//特别注意：因为STM32F4的扇区实在太大，没办法本地保存扇区数据，所以本函数
//          写地址如果非0XFF，那么会先擦除整个扇区且不保存扇区数据。所以
//          写非0XFF的地址，将导致整个扇区数据丢失。建议写之前确保扇区里
//          没有重要数据，最好是整个扇区先擦除了，然后慢慢往后写
void FLASH_Write(u32 WriteAddr,             //写起始地址，必须为4的倍数
                 char pBuffer[],            //数据指针
                 u8 Len)                    //数据字节数
{
    u32 addrx = WriteAddr;                  //写入的起始地址
    u32 endaddr = WriteAddr+Len;            //写入的结束地址

    if( addrx<ADDR_FLASH_SECTOR_2           //低扇区不让写
            ||endaddr>=ADDR_FLASH_SECTOR_8        //我们使用的板子flash只有512kB
            ||addrx%4 )                           //flash编程要求首地址必须被4整除
        return;

    FLASH_Unlock();                         //解锁
    FLASH_DataCacheCmd(DISABLE);            //FLASH擦除期间,必须禁止数据缓存

    while(addrx < endaddr)		            //flash编程必须在0xFFFFFFFF处编程,否则需要全扇区擦除才可以在该位置编程
    {
        if( *(u32*)addrx != 0XFFFFFFFF )
        {
            if(FLASH_EraseSector(GetFlashSector(addrx), VoltageRange_3) != FLASH_COMPLETE)    //发生错误则退出
            {
                FLASH_DataCacheCmd(ENABLE); //FLASH擦除结束,开启数据缓存
                FLASH_Lock();               //上锁
                return;
            }
        }
        else
        {
            addrx += 4;
        }
    }

    while(WriteAddr < endaddr)              //写数据
    {
        if(FLASH_ProgramByte(WriteAddr, *pBuffer) != FLASH_COMPLETE) break; //写入数据，若异常则退出

        WriteAddr++;
        pBuffer++;
    }

    FLASH_DataCacheCmd(ENABLE);             //FLASH擦除结束,开启数据缓存
    FLASH_Lock();                           //上锁
}

u8 FLASH_printf(u32 WriteAddr,              //写起始地址，必须为4的倍数
                const char* fmt,            //数据格式字符串
                ...)                        //其他参数
{
    va_list ap;
    va_start(ap, fmt);
    char FLASH_Buf[255] = {0};//默认最大256
    u8 FLASH_Len = (u8)vsprintf(FLASH_Buf, fmt, ap);	    //将格式化数据打到USART6_DMA_Buf字符串上，返回格式化长度

    FLASH_Write(WriteAddr, FLASH_Buf, FLASH_Len);

    return FLASH_Len;
}

//保存FLASH数据
void FLASH_Save(void)
{
    u32 addr = FLASH_DEFAULT_SAVE_ADDR;

    for(u8 i=0; i<sizeof(FLASH_Data)/sizeof(ST_FLASH_DATA); i++)
    {
        addr += FLASH_printf(addr,"%s=%d;",FLASH_Data[i].str,FLASH_Data[i].d);
        while(addr%4!=0) addr++;    //地址必须是4的倍数
    }
}

//重载FLASH数据，在FLASH_Data[]中注册的数据即使修改也不会丢失数据
void FLASH_Reload(void)
{
    char* ptemp = NULL;
    char FLASH_Buf[255] = {0};  //默认最大256

    FLASH_Read(FLASH_DEFAULT_SAVE_ADDR, FLASH_Buf, sizeof(FLASH_Buf)); //读取默认区数据

    for(u8 i=0; i<sizeof(FLASH_Data)/sizeof(ST_FLASH_DATA); i++)
    {
        ptemp = strstr(FLASH_Buf, FLASH_Data[i].str);
        if(ptemp)               //若找到该字符串则重载数据，找不到则丢弃
        {
            ptemp = strchr(ptemp, '=');
            FLASH_Data[i].d = atoi(ptemp+1);
        }
    }

    FLASH_Save();               //保存数据
}


