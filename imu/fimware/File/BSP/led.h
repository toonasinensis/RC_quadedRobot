#ifndef _LED_H
#define _LED_H

#include "main.h"
#include "stm32f4xx_gpio.h"
/*--------------------------------------------------
	LED0	 PA4	  '0' is on, '1' is off
	LED1	 PA5	  '0' is on, '1' is off
	LED2     PA6      '0' is on, '1' is off
--------------------------------------------------*/
void LED_Configuration(void);
void OUT_Configuration(void);

#define  LED_YELLOW_ON()      	GPIO_ResetBits(GPIOC,  GPIO_Pin_13)
#define  LED_YELLOW_OFF()     	GPIO_SetBits(GPIOC,    GPIO_Pin_13)
#define  LED_YELLOW_TOGGLE()  	GPIO_ToggleBits(GPIOC, GPIO_Pin_13)

#define  LED_GREEN_ON()      	GPIO_ResetBits(GPIOC,  GPIO_Pin_14)
#define  LED_GREEN_OFF()     	GPIO_SetBits(GPIOC,    GPIO_Pin_14)
#define  LED_GREEN_TOGGLE()  	GPIO_ToggleBits(GPIOC, GPIO_Pin_14)

#define  LED_BLUE_ON()      	GPIO_ResetBits(GPIOC,  GPIO_Pin_15)
#define  LED_BLUE_OFF()     	GPIO_SetBits(GPIOC,    GPIO_Pin_15)
#define  LED_BLUE_TOGGLE()  	GPIO_ToggleBits(GPIOC, GPIO_Pin_15)

#define  OUT_1_0V()      		GPIO_ResetBits(GPIOA,  GPIO_Pin_8)
#define  OUT_1_5V()     		GPIO_SetBits(GPIOA,    GPIO_Pin_8)
#define  OUT_1_TOGGLE()  		GPIO_ToggleBits(GPIOA, GPIO_Pin_8)

#define  OUT_2_0V()      		GPIO_ResetBits(GPIOA,  GPIO_Pin_9)
#define  OUT_2_5V()     		GPIO_SetBits(GPIOA,    GPIO_Pin_9)
#define  OUT_2_TOGGLE()  		GPIO_ToggleBits(GPIOA, GPIO_Pin_9)

#endif
