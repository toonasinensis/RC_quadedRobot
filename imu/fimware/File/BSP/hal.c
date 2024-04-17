#include "adc.h"
#include "can.h"
#include "spi.h"
#include "hal.h"
#include "main.h"
#include "time.h"
#include "led.h"
#include "time.h"
#include "usart.h"
#include "systick.h"
#include "icm20602_driver.h"
/****************************************************************************************************
函数名称: BSP_Init()
函数功能: BSP初始化
输入参数: 无
返回参数: 无
备   注: 中断优先级 TIM1 0,0
                    TIM3 0,1
					          DMA2 0,2
****************************************************************************************************/
bool InitCompleteFlag = FALSE;

extern MAIN_SEND_DATA mainsenddata;
extern ST_MAINCONTROL G_ST_MainControl;

void BSP_Init(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);///
    SysTick_Configuration();
    LED_Configuration();///
    USART3_Configuration();
    TIM1_Configuration();
    TIM_SetCompare3(TIM1,500);
    TIM_SetCompare4(TIM1,500);
    TIM2_Configuration();
    TIM3_PWM_Init();
    TIM5_TimeMeasure_Init();

    SPI_Configuration();
    CAN1_Configuration();
//CAN2_Configuration();
    while(icm20602_init());
    InitCompleteFlag = TRUE;
    delay_ms(10);

    delay_ms(1000);			//防止陀螺仪漂移
}
