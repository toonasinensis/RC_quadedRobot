#include "laser.h"


/*-------------------------------
����			|	PC9			|
������		|	0����1��	|
---------------------------------
����			|	�����		|
-------------------------------*/

/*--------------------------------------------------------------------------
�������ܣ���׼����ײ�����
��    ע����PC9����'0'��'off'��'RESET'��'1'��'on'��'SET'
--------------------------------------------------------------------------*/
void Laser_Configuration(void)
{
    GPIO_InitTypeDef gpio;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��PC�˿�ʱ��

    gpio.GPIO_Pin 	= GPIO_Pin_10;
    gpio.GPIO_Mode 	= GPIO_Mode_OUT;	//���ģʽ
    gpio.GPIO_Speed = GPIO_Speed_2MHz;  //IO���ٶ�Ϊ2MHz
    gpio.GPIO_OType = GPIO_OType_PP;	//�������
    GPIO_Init(GPIOA, &gpio);			//�����趨������ʼ��GPIOC

    LASER_OFF();
}









