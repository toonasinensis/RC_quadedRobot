#ifndef __STM32F4xx_IT_H__
#define __STM32F4xx_IT_H__



#include "main.h"
#include "global_declare.h"



void SysTick_Handler(void);

void HardFault_Handler(void);

void CAN1_TX_IRQHandler(void);
void CAN1_RX0_IRQHandler(void);
void CAN2_TX_IRQHandler(void);
void CAN2_RX0_IRQHandler(void);

void USART3_IRQHandler(void);


#endif
