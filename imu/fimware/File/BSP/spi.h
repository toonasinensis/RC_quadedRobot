#ifndef __SPI_H__
#define __SPI_H__

#include "main.h"
#include "stm32f4xx_spi.h"

//PC7 	CSB1
//PB12 	CSB2
//PC9	PS£¨¹Ì¶¨ÎªGND£©

#define Open_SPIx_CSB1_PIN                   GPIO_Pin_4
#define Open_SPIx_CSB1_PORT                  GPIOC
#define Open_SPIx_CSB1_GPIO_CLK              RCC_AHB1Periph_GPIOC

#define Open_SPIx_CSB2_PIN                   GPIO_Pin_5
#define Open_SPIx_CSB2_PORT                  GPIOC
#define Open_SPIx_CSB2_GPIO_CLK              RCC_AHB1Periph_GPIOC

#define Open_SPIx                           SPI1
#define Open_SPIx_CLK                       RCC_APB2Periph_SPI1
#define Open_SPIx_CLK_INIT                  RCC_APB2PeriphClockCmd
#define Open_SPIx_IRQn                      SPI1_IRQn
#define Open_SPIx_IRQHANDLER                SPI1_IRQHandler

#define Open_SPIx_SCK_PIN                   GPIO_Pin_5
#define Open_SPIx_SCK_GPIO_PORT             GPIOA
#define Open_SPIx_SCK_GPIO_CLK              RCC_AHB1Periph_GPIOA
#define Open_SPIx_SCK_SOURCE                GPIO_PinSource5
#define Open_SPIx_SCK_AF                    GPIO_AF_SPI1

#define Open_SPIx_MISO_PIN                  GPIO_Pin_6
#define Open_SPIx_MISO_GPIO_PORT            GPIOA
#define Open_SPIx_MISO_GPIO_CLK             RCC_AHB1Periph_GPIOA
#define Open_SPIx_MISO_SOURCE               GPIO_PinSource6
#define Open_SPIx_MISO_AF                   GPIO_AF_SPI1

#define Open_SPIx_MOSI_PIN                  GPIO_Pin_7
#define Open_SPIx_MOSI_GPIO_PORT            GPIOA
#define Open_SPIx_MOSI_GPIO_CLK             RCC_AHB1Periph_GPIOA
#define Open_SPIx_MOSI_SOURCE               GPIO_PinSource7
#define Open_SPIx_MOSI_AF                   GPIO_AF_SPI1

void SPI_Configuration(void);
USHORT16 spi1_read_write_byte(USHORT16 txc);

#define  GYRO_CS_H    GPIO_SetBits(Open_SPIx_CSB2_PORT, Open_SPIx_CSB2_PIN)//GPIOB   GPIO_Pin_12
#define  GYRO_CS_L    GPIO_ResetBits(Open_SPIx_CSB2_PORT, Open_SPIx_CSB2_PIN)//GPIOB   GPIO_Pin_12

#define  ACC_CS_H    GPIO_SetBits(Open_SPIx_CSB1_PORT, Open_SPIx_CSB1_PIN)//GPIOC  GPIO_Pin_8
#define  ACC_CS_L    GPIO_ResetBits(Open_SPIx_CSB1_PORT, Open_SPIx_CSB1_PIN)//GPIOC  GPIO_Pin_8


#endif

