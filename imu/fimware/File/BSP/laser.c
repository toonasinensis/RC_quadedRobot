#include "laser.h"


/*-------------------------------
引脚			|	PC9			|
输出情况		|	0暗，1亮	|
---------------------------------
功能			|	激光笔		|
-------------------------------*/

/*--------------------------------------------------------------------------
函数功能：瞄准激光底层配置
备    注：对PC9而言'0'即'off'即'RESET'；'1'即'on'即'SET'
--------------------------------------------------------------------------*/
void Laser_Configuration(void)
{
    GPIO_InitTypeDef gpio;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能PC端口时钟

    gpio.GPIO_Pin 	= GPIO_Pin_10;
    gpio.GPIO_Mode 	= GPIO_Mode_OUT;	//输出模式
    gpio.GPIO_Speed = GPIO_Speed_2MHz;  //IO口速度为2MHz
    gpio.GPIO_OType = GPIO_OType_PP;	//推挽输出
    GPIO_Init(GPIOA, &gpio);			//根据设定参数初始化GPIOC

    LASER_OFF();
}









