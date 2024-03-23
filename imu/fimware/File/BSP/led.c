#include "stm32f4xx.h"
#include "led.h"
/*************************************************************************
函 数 名：LED_Configuration(void)
函数功能：LED指示灯底层配置
备    注：PC13:YELLOW	PC14:GREEN	PC15;BLUE		'0' is on,'1' is off
*************************************************************************/
void LED_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    //LED ——复用推挽输出 PA8
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

	LED_YELLOW_ON();
    LED_GREEN_OFF();
    LED_BLUE_OFF();
}
/*************************************************************************
函 数 名：OUT_Configuration(void)
函数功能：相机5V信号输出底层配置
备    注：PA8:OUT_1	PA9:OUT_2						'1' is 5V,'0' is 0V
*************************************************************************/
void OUT_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    //LED ——复用推挽输出 PA8
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

	OUT_1_0V();
	OUT_2_0V();
}

