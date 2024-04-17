#include "stm32f4xx_it.h"

#include "global_declare.h"
/*-------------------------------------------------------
中断服务函数  | SysTick_Handler    | HardFault_Handler   |
---------------------------------------------------------
中断优先级    |                    |                     |
---------------------------------------------------------

----------------------------------------------------------------------------------------------------
中断服务函数  | USART1_IRQHandler  | USART3_IRQHandler  | USART3_IRQHandler   | USART3_IRQHandler   |
----------------------------------------------------------------------------------------------------
中断优先级    | 抢0子3             | 抢0子3             | 抢0子3               | 抢0子3              |
---------------------------------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------
中断产生机制：1ms进入一次中断
函数功能：将SysTick（系统滴答定时器）作为操作系统的时钟
--------------------------------------------------------------------------*/
void SysTick_Handler(void)
{
    static u16 i = 0;
    i++;

    if(i >= 100)
    {
        i = 0;
        //接收不到，开启安全模式
        Calc_System_Monitor_fps(USART4_Xms);
        if(system_monitor.USART4_Xms_fps == 0)
        {
            for(u8 i=0; i<UA4RxDMAbuf_LEN-3; i++) UA4RxDMAbuf[i] = 0;
            MainControl_Rx_Protocol();
        }
    }
    system_monitor.System_cnt++;
}

/*--------------------------------------------------------------------------
中断处理函数名称：void HardFault_Handler(void)
--------------------------------------------------------------------------*/
void HardFault_Handler(void)
{
    while(1);
}

/*--------------------------------------------------------------------------
函数功能：USART1_DMA
--------------------------------------------------------------------------*/
void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USART1, USART_IT_IDLE) == SET)
    {
        USART1->SR;
        USART1->DR;		//先读SR后读DR才能清除IDLE中断标志位

        if(strstr(DistanceMeasureUA1Buf, "mm"))
        {
            X_Axis_Distance = atof(DistanceMeasureUA1Buf);
            system_monitor.USART1_cnt++;
        }
        DMA_Cmd(DMA2_Stream2, DISABLE);         //设置当前计数值前先禁用DMA
        memset(DistanceMeasureUA1Buf, 0, 10);   //清空数组
        DMA2_Stream2->NDTR = 10;                //设置当前待发的数据的数量:Number of Data units to be TRansferred
        DMA_Cmd(DMA2_Stream2, ENABLE);          //启用串口DMA接收
    }
}


/*--------------------------------------------------------------------------
函数功能：USART2_DMA
--------------------------------------------------------------------------*/
void USART2_IRQHandler(void)
{
    if(USART_GetITStatus(USART2, USART_IT_IDLE) == SET)
    {
        USART2->SR;
        USART2->DR;		//先读SR后读DR才能清除IDLE中断标志位

        if(strstr(DistanceMeasureUA2Buf, "mm"))
        {
            Y_Axis_Distance = atof(DistanceMeasureUA2Buf);
            system_monitor.USART2_cnt++;
        }
        DMA_Cmd(DMA1_Stream5, DISABLE);         //设置当前计数值前先禁用DMA
        memset(DistanceMeasureUA2Buf, 0, 10);   //清空数组
        DMA1_Stream5->NDTR = 10;                //设置当前待发的数据的数量:Number of Data units to be TRansferred
        DMA_Cmd(DMA1_Stream5, ENABLE);          //启用串口DMA接收
    }
}


/*--------------------------------------------------------------------------
函数功能：USART3_DMA
--------------------------------------------------------------------------*/
void USART3_IRQHandler(void)
{
    if(USART_GetITStatus(USART3, USART_IT_IDLE) == SET)
    {
        USART3->SR;
        USART3->DR;		//先读SR后读DR才能清除IDLE中断标志位

        if(strstr(DistanceMeasureUA3Buf, "mm"))
        {
            Z_Axis_Distance = atof(DistanceMeasureUA3Buf);
            system_monitor.USART3_cnt++;
        }
        DMA_Cmd(DMA1_Stream1, DISABLE);         //设置当前计数值前先禁用DMA
        memset(DistanceMeasureUA3Buf, 0, 10);   //清空数组
        DMA1_Stream1->NDTR = 10;                //设置当前待发的数据的数量:Number of Data units to be TRansferred
        DMA_Cmd(DMA1_Stream1, ENABLE);          //启用串口DMA接收
    }
}

int n1=0;
/*--------------------------------------------------------------------------
函数功能：接收主控的数据
--------------------------------------------------------------------------*/
void UART4_IRQHandler(void)
{
    if(USART_GetITStatus(UART4, USART_IT_IDLE) == SET)
    {
        UART4->SR;
        UART4->DR;		//先读SR后读DR才能清除IDLE中断标志位
			n1=DMA_GetCurrDataCounter(DMA1_Stream2);
        if( DMA_GetCurrDataCounter(DMA1_Stream2) == UA4RxDMAbuf_LEN )
        {
            MainControl_Rx_Protocol();
            system_monitor.USART4_Xms_cnt++;
            system_monitor.UART4_cnt++;
        }
        else
        {
            DMA_Cmd(DMA1_Stream2, DISABLE);         //设置当前计数值前先禁用DMA
            DMA1_Stream2->NDTR = UA4RxDMAbuf_LEN;   //设置当前待发的数据的数量:Number of Data units to be TRansferred
            DMA_Cmd(DMA1_Stream2, ENABLE);          //启用串口DMA发送
        }
    }
}
/*--------------------------------------------------------------------------
函数功能：
--------------------------------------------------------------------------*/
void USART6_IRQHandler(void)
{
    if(USART_GetITStatus(USART6, USART_IT_IDLE) == SET)
    {
        USART6->SR;
        USART6->DR;		//先读SR后读DR才能清除IDLE中断标志位
        if( DMA_GetCurrDataCounter(DMA1_Stream2) == UA4RxDMAbuf_LEN )
        {
//            MainControl_Rx_Protocol();
            system_monitor.USART6_cnt++;
        }
    }

}
