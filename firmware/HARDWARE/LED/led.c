#include "led.h"

void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_GPIOG_CLK_ENABLE();					//开启GPIOB时钟
	
    GPIO_Initure.Pin=GPIO_PIN_4|GPIO_PIN_3|GPIO_PIN_5;			//PB0，1
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  		//推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;         			//上拉
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;  	//高速
    HAL_GPIO_Init(GPIOG,&GPIO_Initure);     		//初始化GPIOB.0和GPIOB.1
	
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_0,GPIO_PIN_SET);	//PB0置0
	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,GPIO_PIN_SET);	//PB1置1 
}

