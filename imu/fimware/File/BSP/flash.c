#include "flash.h"

ST_FLASH_DATA FLASH_Data[]=
{
    {"YawPosE_Max_history", 0},
    {"YawSpeedE_Max_history", 0},
    {"YawTD.x1_Max_history", 0},
    {"YawTD.x2_Max_history", 0},
};

//��ָ����ַ��ʼ����ָ�����ȵ�����
void FLASH_Read(u32 ReadAddr,               //����ʼ��ַ
                char pBuffer[],             //����ָ��
                u8 Len)                     //�����ֽ���
{
    u8* addr = (u8*)ReadAddr;

    for(u32 i=0; i<Len; i++)
    {
        pBuffer[i] = *addr;
        addr++;
    }
}

//��ȡĳ����ַ���ڵ�flash����
//addr:flash��ַ
//����ֵ:0~11,��addr���ڵ�����
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

//��ָ����ַ�����Կ���������ʼд��ָ�����ȵ�����
//�ر�ע�⣺��ΪSTM32F4������ʵ��̫��û�취���ر����������ݣ����Ա�����
//          д��ַ�����0XFF����ô���Ȳ������������Ҳ������������ݡ�����
//          д��0XFF�ĵ�ַ�������������������ݶ�ʧ������д֮ǰȷ��������
//          û����Ҫ���ݣ���������������Ȳ����ˣ�Ȼ����������д
void FLASH_Write(u32 WriteAddr,             //д��ʼ��ַ������Ϊ4�ı���
                 char pBuffer[],            //����ָ��
                 u8 Len)                    //�����ֽ���
{
    u32 addrx = WriteAddr;                  //д�����ʼ��ַ
    u32 endaddr = WriteAddr+Len;            //д��Ľ�����ַ

    if( addrx<ADDR_FLASH_SECTOR_2           //����������д
            ||endaddr>=ADDR_FLASH_SECTOR_8        //����ʹ�õİ���flashֻ��512kB
            ||addrx%4 )                           //flash���Ҫ���׵�ַ���뱻4����
        return;

    FLASH_Unlock();                         //����
    FLASH_DataCacheCmd(DISABLE);            //FLASH�����ڼ�,�����ֹ���ݻ���

    while(addrx < endaddr)		            //flash��̱�����0xFFFFFFFF�����,������Ҫȫ���������ſ����ڸ�λ�ñ��
    {
        if( *(u32*)addrx != 0XFFFFFFFF )
        {
            if(FLASH_EraseSector(GetFlashSector(addrx), VoltageRange_3) != FLASH_COMPLETE)    //�����������˳�
            {
                FLASH_DataCacheCmd(ENABLE); //FLASH��������,�������ݻ���
                FLASH_Lock();               //����
                return;
            }
        }
        else
        {
            addrx += 4;
        }
    }

    while(WriteAddr < endaddr)              //д����
    {
        if(FLASH_ProgramByte(WriteAddr, *pBuffer) != FLASH_COMPLETE) break; //д�����ݣ����쳣���˳�

        WriteAddr++;
        pBuffer++;
    }

    FLASH_DataCacheCmd(ENABLE);             //FLASH��������,�������ݻ���
    FLASH_Lock();                           //����
}

u8 FLASH_printf(u32 WriteAddr,              //д��ʼ��ַ������Ϊ4�ı���
                const char* fmt,            //���ݸ�ʽ�ַ���
                ...)                        //��������
{
    va_list ap;
    va_start(ap, fmt);
    char FLASH_Buf[255] = {0};//Ĭ�����256
    u8 FLASH_Len = (u8)vsprintf(FLASH_Buf, fmt, ap);	    //����ʽ�����ݴ�USART6_DMA_Buf�ַ����ϣ����ظ�ʽ������

    FLASH_Write(WriteAddr, FLASH_Buf, FLASH_Len);

    return FLASH_Len;
}

//����FLASH����
void FLASH_Save(void)
{
    u32 addr = FLASH_DEFAULT_SAVE_ADDR;

    for(u8 i=0; i<sizeof(FLASH_Data)/sizeof(ST_FLASH_DATA); i++)
    {
        addr += FLASH_printf(addr,"%s=%d;",FLASH_Data[i].str,FLASH_Data[i].d);
        while(addr%4!=0) addr++;    //��ַ������4�ı���
    }
}

//����FLASH���ݣ���FLASH_Data[]��ע������ݼ�ʹ�޸�Ҳ���ᶪʧ����
void FLASH_Reload(void)
{
    char* ptemp = NULL;
    char FLASH_Buf[255] = {0};  //Ĭ�����256

    FLASH_Read(FLASH_DEFAULT_SAVE_ADDR, FLASH_Buf, sizeof(FLASH_Buf)); //��ȡĬ��������

    for(u8 i=0; i<sizeof(FLASH_Data)/sizeof(ST_FLASH_DATA); i++)
    {
        ptemp = strstr(FLASH_Buf, FLASH_Data[i].str);
        if(ptemp)               //���ҵ����ַ������������ݣ��Ҳ�������
        {
            ptemp = strchr(ptemp, '=');
            FLASH_Data[i].d = atoi(ptemp+1);
        }
    }

    FLASH_Save();               //��������
}


