#include "bsp_init.h"

/*-----------------------------------------------------------------------
�������ܣ���ʼ�����еײ�����
-----------------------------------------------------------------------*/
void BSP_Init(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    /*-------------------------------SysTick----------------------------*/
    SysTick_Configuration();

    /*-------------------------------GPIO-------------------------------*/
   LED_Configuration();            //LED
	 OUT_Configuration();			//�������,5V���,��TIM1_PWM_Init()������ͬ����ΪIO�����

    /*-------------------------------TIM--------------------------------*/
//	TIM1_PWM_Init();				//������ƣ���OUT_Configuration()������ͬ����ΪPWM���
	  TIM2_3_Configuration();			//ETR��ʼ��
    TIM4_Configuration();           //���PWM��ʼ��
//	Servo_Control(Servo_Open);		//���ص��գ�����ʹ��
    TIM5_Configuration();           //ʱ��������OSʹ��
//	TIM12_PWM_Init();               //�¶ȿ��ƣ�Ŀǰ���Բ��ȶ����ݲ�ʹ��
    delay_ms(200);
	
    /*-------------------------------USART------------------------------*/
//	UA1_Configuration();            //X���࣬����1����ȡ�������ң����ʹ��
//	UA2_Configuration();            //Y����
//	UA3_Configuration();            //Z����
	USART1_Configuration();			//����ʹ��
	USART2_Configuration();
	USART3_Configuration();
	UART4_Configuration();
	USART6_Configuration();
  delay_ms(200);
	
    /*-------------------------------CAN--------------------------------*/
//	CAN1_Configuration();			//����ʹ��
//	CAN2_Configuration();
	
    /*-------------------------------WWDG-------------------------------*/
//	WWDG_Configuration();			//���Դ�ϵ�ʱ�رգ��ϳ�ʱ��

    /*-------------------------------SPI--------------------------------*/
    SPI_Configuration();

    /*-------------------------------��ʼ�����--------------------------*/
    while(bmi088_init())            //�ȴ���ʼ�����
    {
        delay_ms(1);
    }
}
