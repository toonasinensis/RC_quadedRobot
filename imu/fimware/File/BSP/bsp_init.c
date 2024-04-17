#include "bsp_init.h"

/*-----------------------------------------------------------------------
函数功能：初始化所有底层配置
-----------------------------------------------------------------------*/
void BSP_Init(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    /*-------------------------------SysTick----------------------------*/
    SysTick_Configuration();

    /*-------------------------------GPIO-------------------------------*/
   LED_Configuration();            //LED
	 OUT_Configuration();			//相机控制,5V输出,与TIM1_PWM_Init()功能相同，此为IO口输出

    /*-------------------------------TIM--------------------------------*/
//	TIM1_PWM_Init();				//相机控制，与OUT_Configuration()功能相同，此为PWM输出
	  TIM2_3_Configuration();			//ETR初始化
    TIM4_Configuration();           //舵机PWM初始化
//	Servo_Control(Servo_Open);		//开关弹舱，按需使用
    TIM5_Configuration();           //时间评估，OS使用
//	TIM12_PWM_Init();               //温度控制，目前测试不稳定，暂不使用
    delay_ms(200);
	
    /*-------------------------------USART------------------------------*/
//	UA1_Configuration();            //X轴测距，串口1接受取反，配合遥控器使用
//	UA2_Configuration();            //Y轴测距
//	UA3_Configuration();            //Z轴测距
	USART1_Configuration();			//按需使用
	USART2_Configuration();
	USART3_Configuration();
	UART4_Configuration();
	USART6_Configuration();
  delay_ms(200);
	
    /*-------------------------------CAN--------------------------------*/
//	CAN1_Configuration();			//按需使用
//	CAN2_Configuration();
	
    /*-------------------------------WWDG-------------------------------*/
//	WWDG_Configuration();			//调试打断点时关闭，上场时打开

    /*-------------------------------SPI--------------------------------*/
    SPI_Configuration();

    /*-------------------------------初始化情况--------------------------*/
    while(bmi088_init())            //等待初始化完毕
    {
        delay_ms(1);
    }
}
