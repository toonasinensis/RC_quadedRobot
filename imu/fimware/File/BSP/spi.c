#include "spi.h"

void SPI_Configuration(void)
{
    SPI_InitTypeDef SPI_InitStruct;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB2PeriphClockCmd(Open_SPIx_SCK_GPIO_CLK | Open_SPIx_MISO_GPIO_CLK | Open_SPIx_MOSI_GPIO_CLK,ENABLE); // ��SPI�õ���IO�ܽŹ��ڵ�ʱ����
    Open_SPIx_CLK_INIT(Open_SPIx_CLK,ENABLE);   //��ʼ��SPI��ʱ�ӣ�ע�⣬SPI2��SPI3�ú���RCC_APB1PeriphClockCmd��SPI1�ú���RCC_APB2PeriphClockCmd
    //ע�⣬��������ʵ�ֵĹ����ǲ�һ���ġ�SPI��Ϊ��Ƭ�������裬��Ҫʱ�ӽ��й�����IO�ǲ���SPI��һ���֣�ֻ�ǿ��Ը��ó�ĳһ·SPI��IO��Ҳ���Լ���ʱ�ӡ�

    //�˿ڸ���
    GPIO_PinAFConfig(Open_SPIx_SCK_GPIO_PORT, Open_SPIx_SCK_SOURCE,  Open_SPIx_SCK_AF);
    GPIO_PinAFConfig(Open_SPIx_MISO_GPIO_PORT, Open_SPIx_MISO_SOURCE, Open_SPIx_MOSI_AF);
    GPIO_PinAFConfig(Open_SPIx_MOSI_GPIO_PORT, Open_SPIx_MOSI_SOURCE, Open_SPIx_MOSI_AF);

    // �˿ڸ������ã�һ����˵��SPI��ʹ��Ӱ�첢���Ǻܴ�
    GPIO_InitStructure.GPIO_Pin = Open_SPIx_SCK_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(Open_SPIx_SCK_GPIO_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = Open_SPIx_MISO_PIN;
    GPIO_Init(Open_SPIx_MISO_GPIO_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = Open_SPIx_MOSI_PIN;
    GPIO_Init(Open_SPIx_MOSI_GPIO_PORT, &GPIO_InitStructure);

    // SPI�ľ������ã�ÿһ�����ܹؼ���������Ҫ��ѯ������оƬ��datasheet
    SPI_I2S_DeInit(Open_SPIx);
    SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
    SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStruct.SPI_CRCPolynomial = 7;
    SPI_InitStruct.SPI_DataSize = SPI_DataSize_16b;
    SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
    SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
    SPI_Init(Open_SPIx,&SPI_InitStruct);
    SPI_Cmd(Open_SPIx, ENABLE);

    /* ��ʼ��NSS/Csn �ܽţ���ʼ��Ƭѡ�ܽ�*/
    RCC_AHB1PeriphClockCmd(Open_SPIx_CSB1_GPIO_CLK, ENABLE);
    RCC_AHB1PeriphClockCmd(Open_SPIx_CSB2_GPIO_CLK, ENABLE);

    //CSB1
    GPIO_InitStructure.GPIO_Pin = Open_SPIx_CSB1_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_Init(Open_SPIx_CSB1_PORT, &GPIO_InitStructure);

    //CSB2
    GPIO_InitStructure.GPIO_Pin = Open_SPIx_CSB2_PIN;
    GPIO_Init(Open_SPIx_CSB2_PORT, &GPIO_InitStructure);

    GPIO_SetBits(Open_SPIx_CSB1_PORT, Open_SPIx_CSB1_PIN);
    GPIO_SetBits(Open_SPIx_CSB2_PORT, Open_SPIx_CSB2_PIN);
}


USHORT16 spi1_read_write_byte(USHORT16 txc)
{
    while((SPI1->SR&SPI_SR_TXE)==0);    //�ȴ����ͽ���
    SPI1->DR = txc;
    while((SPI1->SR&SPI_SR_RXNE)==0);   //�ȴ����ս���
    return SPI1->DR;
}
