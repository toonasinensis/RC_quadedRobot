#ifndef _USART_H_
#define _USART_H_

#include "main.h"
#include "stm32f4xx_dma.h"
#include "usart_protocol.h"

void UART4_DMA_printf(const char* fmt, ...);

void USART1_Configuration(void);
void USART2_Configuration(void);
void USART3_Configuration(void);
void UART4_Configuration(void);
void USART6_Configuration(void);

#endif
