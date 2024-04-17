#include "systick.h"


/*------------------------------------------------------------------------
函 数 名：SysTick_Configuration
函数功能：初始化SysTick_Configuration配置为1ms中断
------------------------------------------------------------------------*/
void SysTick_Configuration(void)
{
    //SYSTICK分频--1ms的系统时钟中断
    //SystemCoreClock / 10       100ms
    //SystemCoreClock / 1000     1ms
    //SystemCoreClock / 100000   10us
    //SystemCoreClock / 1000000  1us
    while(SysTick_Config(SystemCoreClock/1000));//1ms
    NVIC_SetPriority(SysTick_IRQn,0x00);
}
