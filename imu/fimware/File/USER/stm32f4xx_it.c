#include "stm32f4xx_it.h"

#include "global_declare.h"
/*-------------------------------------------------------
�жϷ�����  | SysTick_Handler    | HardFault_Handler   |
---------------------------------------------------------
�ж����ȼ�    |                    |                     |
---------------------------------------------------------

----------------------------------------------------------------------------------------------------
�жϷ�����  | USART1_IRQHandler  | USART3_IRQHandler  | USART3_IRQHandler   | USART3_IRQHandler   |
----------------------------------------------------------------------------------------------------
�ж����ȼ�    | ��0��3             | ��0��3             | ��0��3               | ��0��3              |
---------------------------------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------
�жϲ������ƣ�1ms����һ���ж�
�������ܣ���SysTick��ϵͳ�δ�ʱ������Ϊ����ϵͳ��ʱ��
--------------------------------------------------------------------------*/
void SysTick_Handler(void)
{
    static u16 i = 0;
    i++;

    if(i >= 100)
    {
        i = 0;
        //���ղ�����������ȫģʽ
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
�жϴ��������ƣ�void HardFault_Handler(void)
--------------------------------------------------------------------------*/
void HardFault_Handler(void)
{
    while(1);
}

/*--------------------------------------------------------------------------
�������ܣ�USART1_DMA
--------------------------------------------------------------------------*/
void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USART1, USART_IT_IDLE) == SET)
    {
        USART1->SR;
        USART1->DR;		//�ȶ�SR���DR�������IDLE�жϱ�־λ

        if(strstr(DistanceMeasureUA1Buf, "mm"))
        {
            X_Axis_Distance = atof(DistanceMeasureUA1Buf);
            system_monitor.USART1_cnt++;
        }
        DMA_Cmd(DMA2_Stream2, DISABLE);         //���õ�ǰ����ֵǰ�Ƚ���DMA
        memset(DistanceMeasureUA1Buf, 0, 10);   //�������
        DMA2_Stream2->NDTR = 10;                //���õ�ǰ���������ݵ�����:Number of Data units to be TRansferred
        DMA_Cmd(DMA2_Stream2, ENABLE);          //���ô���DMA����
    }
}


/*--------------------------------------------------------------------------
�������ܣ�USART2_DMA
--------------------------------------------------------------------------*/
void USART2_IRQHandler(void)
{
    if(USART_GetITStatus(USART2, USART_IT_IDLE) == SET)
    {
        USART2->SR;
        USART2->DR;		//�ȶ�SR���DR�������IDLE�жϱ�־λ

        if(strstr(DistanceMeasureUA2Buf, "mm"))
        {
            Y_Axis_Distance = atof(DistanceMeasureUA2Buf);
            system_monitor.USART2_cnt++;
        }
        DMA_Cmd(DMA1_Stream5, DISABLE);         //���õ�ǰ����ֵǰ�Ƚ���DMA
        memset(DistanceMeasureUA2Buf, 0, 10);   //�������
        DMA1_Stream5->NDTR = 10;                //���õ�ǰ���������ݵ�����:Number of Data units to be TRansferred
        DMA_Cmd(DMA1_Stream5, ENABLE);          //���ô���DMA����
    }
}


/*--------------------------------------------------------------------------
�������ܣ�USART3_DMA
--------------------------------------------------------------------------*/
void USART3_IRQHandler(void)
{
    if(USART_GetITStatus(USART3, USART_IT_IDLE) == SET)
    {
        USART3->SR;
        USART3->DR;		//�ȶ�SR���DR�������IDLE�жϱ�־λ

        if(strstr(DistanceMeasureUA3Buf, "mm"))
        {
            Z_Axis_Distance = atof(DistanceMeasureUA3Buf);
            system_monitor.USART3_cnt++;
        }
        DMA_Cmd(DMA1_Stream1, DISABLE);         //���õ�ǰ����ֵǰ�Ƚ���DMA
        memset(DistanceMeasureUA3Buf, 0, 10);   //�������
        DMA1_Stream1->NDTR = 10;                //���õ�ǰ���������ݵ�����:Number of Data units to be TRansferred
        DMA_Cmd(DMA1_Stream1, ENABLE);          //���ô���DMA����
    }
}

int n1=0;
/*--------------------------------------------------------------------------
�������ܣ��������ص�����
--------------------------------------------------------------------------*/
void UART4_IRQHandler(void)
{
    if(USART_GetITStatus(UART4, USART_IT_IDLE) == SET)
    {
        UART4->SR;
        UART4->DR;		//�ȶ�SR���DR�������IDLE�жϱ�־λ
			n1=DMA_GetCurrDataCounter(DMA1_Stream2);
        if( DMA_GetCurrDataCounter(DMA1_Stream2) == UA4RxDMAbuf_LEN )
        {
            MainControl_Rx_Protocol();
            system_monitor.USART4_Xms_cnt++;
            system_monitor.UART4_cnt++;
        }
        else
        {
            DMA_Cmd(DMA1_Stream2, DISABLE);         //���õ�ǰ����ֵǰ�Ƚ���DMA
            DMA1_Stream2->NDTR = UA4RxDMAbuf_LEN;   //���õ�ǰ���������ݵ�����:Number of Data units to be TRansferred
            DMA_Cmd(DMA1_Stream2, ENABLE);          //���ô���DMA����
        }
    }
}
/*--------------------------------------------------------------------------
�������ܣ�
--------------------------------------------------------------------------*/
void USART6_IRQHandler(void)
{
    if(USART_GetITStatus(USART6, USART_IT_IDLE) == SET)
    {
        USART6->SR;
        USART6->DR;		//�ȶ�SR���DR�������IDLE�жϱ�־λ
        if( DMA_GetCurrDataCounter(DMA1_Stream2) == UA4RxDMAbuf_LEN )
        {
//            MainControl_Rx_Protocol();
            system_monitor.USART6_cnt++;
        }
    }

}
