#include "friction_wheel.h"

/*************************************************************************
�� �� ����TIM1_Configuration
�������ܣ����50Hz
��    ע��TIM1_
          PWMƵ��=168MHz/(Prescaler+1)/(Period+1)
          PWMռ�ձ�=Pulse/Period
*************************************************************************/
void TIM1_Configuration(void)
{
    GPIO_InitTypeDef          gpio;
    TIM_TimeBaseInitTypeDef   tim;
    TIM_OCInitTypeDef         oc;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);//168MHz

    gpio.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11;
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &gpio);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_TIM1);

    //Ƶ��168M/(168*2)/10000=50Hz
    tim.TIM_Prescaler     = 168*2-1;
    tim.TIM_CounterMode   = TIM_CounterMode_Up;
    tim.TIM_Period        = 10000-1;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1,&tim);

    oc.TIM_OCMode       = TIM_OCMode_PWM1;          //���ϼ�����cnt<ccrʱ��Ч
    oc.TIM_OutputState  = TIM_OutputState_Enable;
    oc.TIM_OutputNState = TIM_OutputState_Disable;
    oc.TIM_Pulse        = 400;                      //ռ�ձ�500/10000=5%
    oc.TIM_OCPolarity   = TIM_OCPolarity_High;      //�ߵ�ƽ��Ч
    oc.TIM_OCNPolarity  = TIM_OCPolarity_High;
    oc.TIM_OCIdleState  = TIM_OCIdleState_Reset;
    oc.TIM_OCNIdleState = TIM_OCIdleState_Reset;
    TIM_OC3Init(TIM1, &oc);
    TIM_OC4Init(TIM1, &oc);

    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);

    TIM_CtrlPWMOutputs(TIM1, ENABLE);   //TIM1 PWM���Ҫ��һ�仰
    TIM_Cmd(TIM1, ENABLE);
}
