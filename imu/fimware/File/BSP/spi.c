#include "spi.h"

void SPI_Configuration(void)
{
    SPI_InitTypeDef SPI_InitStruct;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB2PeriphClockCmd(Open_SPIx_SCK_GPIO_CLK | Open_SPIx_MISO_GPIO_CLK | Open_SPIx_MOSI_GPIO_CLK,ENABLE); // 将SPI用到的IO管脚挂在到时钟上
    Open_SPIx_CLK_INIT(Open_SPIx_CLK,ENABLE);   //初始化SPI的时钟，注意，SPI2和SPI3用函数RCC_APB1PeriphClockCmd，SPI1用函数RCC_APB2PeriphClockCmd
    //注意，以上两步实现的功能是不一样的。SPI作为单片机的内设，需要时钟进行工作。IO是不是SPI的一部分，只是可以复用成某一路SPI，IO口也有自己的时钟。

    //端口复用
    GPIO_PinAFConfig(Open_SPIx_SCK_GPIO_PORT, Open_SPIx_SCK_SOURCE,  Open_SPIx_SCK_AF);
    GPIO_PinAFConfig(Open_SPIx_MISO_GPIO_PORT, Open_SPIx_MISO_SOURCE, Open_SPIx_MOSI_AF);
    GPIO_PinAFConfig(Open_SPIx_MOSI_GPIO_PORT, Open_SPIx_MOSI_SOURCE, Open_SPIx_MOSI_AF);

    // 端口复用设置，一般来说对SPI的使用影响并不是很大。
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

    // SPI的具体配置，每一步都很关键。具体需要查询所操作芯片的datasheet
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

    /* 初始化NSS/Csn 管脚，初始化片选管脚*/
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
    while((SPI1->SR&SPI_SR_TXE)==0);    //等待发送结束
    SPI1->DR = txc;
    while((SPI1->SR&SPI_SR_RXNE)==0);   //等待接收结束
    return SPI1->DR;
}
