#include "time.h"
/****************************************************************************************************
函数名称: TIM1_Configuration()
函数功能: 配置TIM1，相机信号电路使用
输入参数: 无
返回参数: 无
备   注:
****************************************************************************************************/
void TIM1_PWM_Init(void)
{
    GPIO_InitTypeDef          gpio;
    TIM_TimeBaseInitTypeDef   tim;
    TIM_OCInitTypeDef         oc;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);//168MHz

    gpio.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9;
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &gpio);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1);

    //频率168M/(168*2)/10000=50Hz
    tim.TIM_Prescaler     = Camera_Prescaler;
    tim.TIM_CounterMode   = TIM_CounterMode_Up;
    tim.TIM_Period        = Camera_Period;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &tim);

    oc.TIM_OCMode       = TIM_OCMode_PWM1;          //向上计数，cnt<ccr时有效
    oc.TIM_OutputState  = TIM_OutputState_Enable;
    oc.TIM_Pulse        = Camera_Pause;				//占空比
    oc.TIM_OCPolarity   = TIM_OCPolarity_High;      //高电平有效
    oc.TIM_OCIdleState  = TIM_OCIdleState_Reset;
    TIM_OC1Init(TIM1, &oc);
    TIM_OC2Init(TIM1, &oc);

    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);

    TIM_CtrlPWMOutputs(TIM1, ENABLE);   //TIM1 PWM输出要多一句话
    TIM_Cmd(TIM1, ENABLE);
}
/****************************************************************************************************
函数名称: TIM2_3_Configuration()
函数功能: 配置TIM2，相机信号电路使用
输入参数: 无
返回参数: 无
备   注:
****************************************************************************************************/
/*TIM2_ETR -- PA15	TIM3_ETR -- PD2*/
void TIM2_3_Configuration(void)
{
	GPIO_InitTypeDef gpio;
	TIM_TimeBaseInitTypeDef tim;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOD, ENABLE);   //使能PA、PD端口时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3,  ENABLE);   //使能TIM2、TIM3时钟,84MHz

	gpio.GPIO_Pin = GPIO_Pin_15;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOA, &gpio);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_TIM2);

	gpio.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOD, &gpio);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_TIM3);
	
	
	tim.TIM_Period = 0xFFFF; 
	tim.TIM_Prescaler = 0; 
	tim.TIM_ClockDivision = TIM_CKD_DIV1; 
	tim.TIM_CounterMode = TIM_CounterMode_Up;  
	
	TIM_TimeBaseInit(TIM2, &tim);
	TIM_ETRClockMode2Config(TIM2, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);  //外部时钟源模式2
	TIM_SetCounter(TIM2,0);
	TIM_Cmd(TIM2, ENABLE);

	TIM_TimeBaseInit(TIM3, &tim);
	TIM_ETRClockMode2Config(TIM3, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted,0);  //外部时钟源模式2
	TIM_SetCounter(TIM3,0);
	TIM_Cmd(TIM3, ENABLE);
}
/****************************************************************************************************
函数名称: TIM12_PWM_Init()
函数功能: 配置TIM12，温控电路使用
输入参数: 无
返回参数: 无
备   注:
****************************************************************************************************/
void TIM12_PWM_Init(void)
{
    GPIO_InitTypeDef        gpio;
    TIM_TimeBaseInitTypeDef tim;
    TIM_OCInitTypeDef       tim_oc;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);	//使能定时器3时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);  //使能GPIO外设和AFIO复用功能模块时钟

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource14,GPIO_AF_TIM14);

    //超级电容加热
    gpio.GPIO_Pin   = GPIO_Pin_14;				//PB14 -- TIM12_CH1
    gpio.GPIO_Mode  = GPIO_Mode_AF;  			//复用推挽输出
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);					//初始化GPIO

    //初始化TIM12
    tim.TIM_Prescaler         = 84-1;                //设置用来作为TIMx时钟频率除数的预分频值
    tim.TIM_Period            = 1000-1;              //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
    tim.TIM_ClockDivision     = 0;                   //设置时钟分割:TDTS = Tck_tim
    tim.TIM_CounterMode       = TIM_CounterMode_Up;  //TIM向上计数模式
    TIM_TimeBaseInit(TIM4, &tim);                    //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

    //初始化TIM12 Channel1 PWM模式 （温控电路）
    tim_oc.TIM_OCMode         = TIM_OCMode_PWM1;        //选择定时器模式:TIM脉冲宽度调制模式1
    tim_oc.TIM_OutputState    = TIM_OutputState_Enable; //比较输出使能
    tim_oc.TIM_OCPolarity     = TIM_OCPolarity_High;    //输出极性:TIM输出比较极性高
    tim_oc.TIM_Pulse          = 0;                      //设置待装入捕获比较寄存器的脉冲值
    TIM_OC1Init(TIM12, &tim_oc);                         //根据T指定的参数初始化外设TIM3 OC1

    TIM_OC1PreloadConfig(TIM12, TIM_OCPreload_Enable);   //使能TIM12在CCR1上的预装载寄存器

    TIM_ARRPreloadConfig(TIM12, ENABLE);                 //使能TIMx在ARR上的预装载寄存器
    TIM_CtrlPWMOutputs(TIM12,ENABLE);                    //MOE 主输出使能
    TIM_Cmd(TIM12, ENABLE);                              //使能TIM12
}
