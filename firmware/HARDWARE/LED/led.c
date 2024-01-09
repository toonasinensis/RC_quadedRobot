#include "led.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32H7开发板
//LED驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2017/6/8
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

//初始化PB0,PB1为输出.并使能这两个口的时钟		    
//LED IO初始化
void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_GPIOB_CLK_ENABLE();					//开启GPIOB时钟
	
    GPIO_Initure.Pin=GPIO_PIN_0|GPIO_PIN_1;			//PB0，1
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  		//推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;         			//上拉
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;  	//高速
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);     		//初始化GPIOB.0和GPIOB.1
	
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);	//PB0置0
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);	//PB1置1 
}

