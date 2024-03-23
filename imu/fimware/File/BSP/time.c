#include "time.h"
/****************************************************************************************************
��������: TIM1_Configuration()
��������: ����TIM1������źŵ�·ʹ��
�������: ��
���ز���: ��
��   ע:
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

    //Ƶ��168M/(168*2)/10000=50Hz
    tim.TIM_Prescaler     = Camera_Prescaler;
    tim.TIM_CounterMode   = TIM_CounterMode_Up;
    tim.TIM_Period        = Camera_Period;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &tim);

    oc.TIM_OCMode       = TIM_OCMode_PWM1;          //���ϼ�����cnt<ccrʱ��Ч
    oc.TIM_OutputState  = TIM_OutputState_Enable;
    oc.TIM_Pulse        = Camera_Pause;				//ռ�ձ�
    oc.TIM_OCPolarity   = TIM_OCPolarity_High;      //�ߵ�ƽ��Ч
    oc.TIM_OCIdleState  = TIM_OCIdleState_Reset;
    TIM_OC1Init(TIM1, &oc);
    TIM_OC2Init(TIM1, &oc);

    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);

    TIM_CtrlPWMOutputs(TIM1, ENABLE);   //TIM1 PWM���Ҫ��һ�仰
    TIM_Cmd(TIM1, ENABLE);
}
/****************************************************************************************************
��������: TIM2_3_Configuration()
��������: ����TIM2������źŵ�·ʹ��
�������: ��
���ز���: ��
��   ע:
****************************************************************************************************/
/*TIM2_ETR -- PA15	TIM3_ETR -- PD2*/
void TIM2_3_Configuration(void)
{
	GPIO_InitTypeDef gpio;
	TIM_TimeBaseInitTypeDef tim;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOD, ENABLE);   //ʹ��PA��PD�˿�ʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3,  ENABLE);   //ʹ��TIM2��TIM3ʱ��,84MHz

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
	TIM_ETRClockMode2Config(TIM2, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);  //�ⲿʱ��Դģʽ2
	TIM_SetCounter(TIM2,0);
	TIM_Cmd(TIM2, ENABLE);

	TIM_TimeBaseInit(TIM3, &tim);
	TIM_ETRClockMode2Config(TIM3, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted,0);  //�ⲿʱ��Դģʽ2
	TIM_SetCounter(TIM3,0);
	TIM_Cmd(TIM3, ENABLE);
}
/****************************************************************************************************
��������: TIM12_PWM_Init()
��������: ����TIM12���¿ص�·ʹ��
�������: ��
���ز���: ��
��   ע:
****************************************************************************************************/
void TIM12_PWM_Init(void)
{
    GPIO_InitTypeDef        gpio;
    TIM_TimeBaseInitTypeDef tim;
    TIM_OCInitTypeDef       tim_oc;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);	//ʹ�ܶ�ʱ��3ʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);  //ʹ��GPIO�����AFIO���ù���ģ��ʱ��

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource14,GPIO_AF_TIM14);

    //�������ݼ���
    gpio.GPIO_Pin   = GPIO_Pin_14;				//PB14 -- TIM12_CH1
    gpio.GPIO_Mode  = GPIO_Mode_AF;  			//�����������
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);					//��ʼ��GPIO

    //��ʼ��TIM12
    tim.TIM_Prescaler         = 84-1;                //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
    tim.TIM_Period            = 1000-1;              //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
    tim.TIM_ClockDivision     = 0;                   //����ʱ�ӷָ�:TDTS = Tck_tim
    tim.TIM_CounterMode       = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
    TIM_TimeBaseInit(TIM4, &tim);                    //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

    //��ʼ��TIM12 Channel1 PWMģʽ ���¿ص�·��
    tim_oc.TIM_OCMode         = TIM_OCMode_PWM1;        //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
    tim_oc.TIM_OutputState    = TIM_OutputState_Enable; //�Ƚ����ʹ��
    tim_oc.TIM_OCPolarity     = TIM_OCPolarity_High;    //�������:TIM����Ƚϼ��Ը�
    tim_oc.TIM_Pulse          = 0;                      //���ô�װ�벶��ȽϼĴ���������ֵ
    TIM_OC1Init(TIM12, &tim_oc);                         //����Tָ���Ĳ�����ʼ������TIM3 OC1

    TIM_OC1PreloadConfig(TIM12, TIM_OCPreload_Enable);   //ʹ��TIM12��CCR1�ϵ�Ԥװ�ؼĴ���

    TIM_ARRPreloadConfig(TIM12, ENABLE);                 //ʹ��TIMx��ARR�ϵ�Ԥװ�ؼĴ���
    TIM_CtrlPWMOutputs(TIM12,ENABLE);                    //MOE �����ʹ��
    TIM_Cmd(TIM12, ENABLE);                              //ʹ��TIM12
}
