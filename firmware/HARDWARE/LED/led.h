#ifndef _LED_H
#define _LED_H
#include "sys.h"

// LED端口定义
#define LED0(n)                                                                \
  (n ? HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_SET)                      \
     : HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_RESET))
#define LED0_Toggle (HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_3)) // LED0输出电平翻转

#define LED1(n)                                                                \
  (n ? HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_SET)                      \
     : HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_RESET))
#define LED0_Toggle (HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_4)) // LED0输出电平翻转

#define LED2(n)                                                                \
  (n ? HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_SET)                      \
     : HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_RESET))
#define LED0_Toggle (HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_5)) // LED0输出电平翻转

void LED_Init(void); // LED初始化函数
#endif
