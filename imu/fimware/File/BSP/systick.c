#include "systick.h"


/*------------------------------------------------------------------------
�� �� ����SysTick_Configuration
�������ܣ���ʼ��SysTick_Configuration����Ϊ1ms�ж�
------------------------------------------------------------------------*/
void SysTick_Configuration(void)
{
    //SYSTICK��Ƶ--1ms��ϵͳʱ���ж�
    //SystemCoreClock / 10       100ms
    //SystemCoreClock / 1000     1ms
    //SystemCoreClock / 100000   10us
    //SystemCoreClock / 1000000  1us
    while(SysTick_Config(SystemCoreClock/1000));//1ms
    NVIC_SetPriority(SysTick_IRQn,0x00);
}
