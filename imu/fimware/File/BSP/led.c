#include "stm32f4xx.h"
#include "led.h"
/*************************************************************************
�� �� ����LED_Configuration(void)
�������ܣ�LEDָʾ�Ƶײ�����
��    ע��PC13:YELLOW	PC14:GREEN	PC15;BLUE		'0' is on,'1' is off
*************************************************************************/
void LED_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    //LED ��������������� PA8
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
�� �� ����OUT_Configuration(void)
�������ܣ����5V�ź�����ײ�����
��    ע��PA8:OUT_1	PA9:OUT_2						'1' is 5V,'0' is 0V
*************************************************************************/
void OUT_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    //LED ��������������� PA8
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

	OUT_1_0V();
	OUT_2_0V();
}

