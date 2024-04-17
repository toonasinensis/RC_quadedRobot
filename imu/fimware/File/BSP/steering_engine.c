#include "steering_engine.h"

//u32 SteeringEngine_Prescaler = 168-1;
//u32 SteeringEngine_Period = 1667-1;

//u32 SteeringEnginePWM_High = 820;
//u32 SteeringEnginePWM_Low = 820;

//    #define SteeringEngine_Prescaler 168-1
//    #define SteeringEngine_Period    10000-1

//    #define SteeringEnginePWM_High   820
//    #define SteeringEnginePWM_Low    1250

//PA0----TIM2CH1----SteeringEnginePWM1----舵机
//PA1----TIM2CH2----SteeringEnginePWM2----舵机

//0.5ms--------------0度
//1.0ms-------------45度
//1.5ms-------------90度
//2.0ms------------135度
//2.5ms------------180度

/*--------------------------------------------------------------------------
函数功能：输出50Hz
备    注：TIM1 TIM2
          PWM频率=84MHz/(Prescaler+1)/(Period+1)
          PWM占空比=Pulse/Period
--------------------------------------------------------------------------*/


void TIM4_Configuration(void)
{
    GPIO_InitTypeDef          gpio;
    TIM_TimeBaseInitTypeDef   tim;
    TIM_OCInitTypeDef         oc;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//84MHz

	gpio.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOB, &gpio);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM4);

    //频率84M/(168)/10000=50Hz
    tim.TIM_Prescaler     = SteeringEngine_Prescaler;
    tim.TIM_CounterMode   = TIM_CounterMode_Up;
    tim.TIM_Period        = SteeringEngine_Period;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4,&tim);

    oc.TIM_OCMode       = TIM_OCMode_PWM1;          //向上计数，cnt<ccr时有效
    oc.TIM_OutputState  = TIM_OutputState_Enable;
    oc.TIM_Pulse        = SteeringEnginePWM_High;   //占空比
    oc.TIM_OCPolarity   = TIM_OCPolarity_High;      //高电平有效
    oc.TIM_OCIdleState  = TIM_OCIdleState_Reset;
    TIM_OC3Init(TIM4, &oc);
    TIM_OC4Init(TIM4, &oc);

    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM4, ENABLE);

    TIM_CtrlPWMOutputs(TIM4, ENABLE);   //TIM4 PWM输出要多一句话
    TIM_Cmd(TIM4, ENABLE);
}


/*----------------------------------------------------------------------------------------
函数功能：开关弹仓
----------------------------------------------------------------------------------------*/
void Servo_Control(ServoState state)
{
    if(state == Servo_Open)
    {
        SteeringEnginePWM1 = SteeringEnginePWM_High;
        SteeringEnginePWM2 = SteeringEnginePWM_High;
    }
    else
    {
        SteeringEnginePWM1 = SteeringEnginePWM_Low;
        SteeringEnginePWM2 = SteeringEnginePWM_Low;
    }
}


