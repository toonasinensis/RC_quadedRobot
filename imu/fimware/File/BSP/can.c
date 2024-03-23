#include "can.h"



/*-------------------------------------------------------
CAN1			|	Rx              |	Tx			    |
---------------------------------------------------------
引脚			|	PA11		 	|	PA2		    	|
中断优先级		|	抢0子3		    |	抢1子1		    |
波特率			|	42M			    |	42M			    |
---------------------------------------------------------
功能			|   底盘电机、裁判系统、超级电容	    |
---------------------------------------------------------

---------------------------------------------------------
CAN2			|	Rx              |	Tx			    |
---------------------------------------------------------
引脚			|	PB12            |   PB13		        |
中断优先级		|	抢0子4		    |	抢1子2		    |
波特率			|	42M			    |	42M			    |
---------------------------------------------------------
功能			|	拨弹电机、pitch电机、Yaw电机	    |
-------------------------------------------------------*/


/*--------------------------------------------------------------------------
函数功能：初始化CAN1配置为1M波特率
备    注：PB8(CAN1_RX);PB9(CAN1_TX)
          BaudRate = 42MHz/Prescaler*(1+BS1+BS2)
--------------------------------------------------------------------------*/
void CAN1_Configuration(void)
{
    GPIO_InitTypeDef       gpio;
    NVIC_InitTypeDef       nvic;
    CAN_InitTypeDef        can1;
    CAN_FilterInitTypeDef  can1_fliter;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);   //使能PB端口时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,  ENABLE);   //使能CAN1时钟,42MHz

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);

    gpio.GPIO_Pin   = GPIO_Pin_11 | GPIO_Pin_12;
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_Init(GPIOA, &gpio);

    nvic.NVIC_IRQChannel                    = CAN1_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority  = 0;        //抢占优先级
    nvic.NVIC_IRQChannelSubPriority         = 3;        //子优先级
    nvic.NVIC_IRQChannelCmd                 = ENABLE;   //IRQ通道使能
    NVIC_Init(&nvic);   //根据指定的参数初始化NVIC寄存器

    /****************************************CAN单元初始化*****************************************/
    CAN_DeInit(CAN1);
    CAN_StructInit(&can1);
    /*CAN主控制寄存器器（CAN_MCR）*/
    can1.CAN_TTCM = DISABLE;        //时间触发通信模式
    can1.CAN_ABOM = DISABLE;        //自动的总线关闭管理
    can1.CAN_AWUM = DISABLE;        //自动唤醒模式
    can1.CAN_NART = DISABLE;        //禁止自动重发送
    can1.CAN_RFLM = DISABLE;        //接收FIFO锁定模式
    can1.CAN_TXFP = ENABLE;         //发送FIFIO优先级
    /*CAN位时序寄存器（CAN_BTR）*/
    /*CAN1波特率=42MHz/3/(9+4+1)=1MHz*/
    can1.CAN_Mode = CAN_Mode_Normal;//普通模式
    can1.CAN_SJW  = CAN_SJW_1tq;    //重新同步跳跃宽度
    can1.CAN_BS1  = CAN_BS1_9tq;    //时间段1
    can1.CAN_BS2  = CAN_BS2_4tq;    //时间段2
    can1.CAN_Prescaler = 3;         //分频系数
    CAN_Init(CAN1, &can1);          //根据指定的参数初始化CAN寄存器
    /****************************************CAN过滤器初始化****************************************/
    can1_fliter.CAN_FilterNumber         = 0;                       //过滤器0
    can1_fliter.CAN_FilterMode           = CAN_FilterMode_IdMask;   //标识符过滤器为掩码模式
    can1_fliter.CAN_FilterScale          = CAN_FilterScale_32bit;   //一个32位标识符过滤器
    can1_fliter.CAN_FilterIdHigh         = 0x0000;                  //32位标识符过滤器的高16位
    can1_fliter.CAN_FilterIdLow          = 0x0000;                  //32位标识符过滤器的低16位
    can1_fliter.CAN_FilterMaskIdHigh     = 0x0000;                  //过滤器掩码高16位
    can1_fliter.CAN_FilterMaskIdLow      = 0x0000;                  //过滤器掩码低16位
    can1_fliter.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;        //过滤器0关联到FIFO0
    can1_fliter.CAN_FilterActivation     = ENABLE;                  //激活过滤器
    CAN_FilterInit(&can1_fliter);             //根据指定的参数初始化CAN_Filter寄存器

    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);  //接收中断:FIFO 0消息挂起中断
}


/*--------------------------------------------------------------------------
函数功能：将CAN2配置为1M波特率
备    注：PB5(CAN2_RX);PB6(CAN2_TX)
          BaudRate = 42MHz/Prescaler*(1+BS1+BS2)
--------------------------------------------------------------------------*/
void CAN2_Configuration(void)
{
    GPIO_InitTypeDef       gpio;
    NVIC_InitTypeDef       nvic;
    CAN_InitTypeDef        can2;
    CAN_FilterInitTypeDef  can2_filter;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);   //使能PB端口时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,  ENABLE);   //使能CAN1时钟*注意*若单独使用can2,需先使能can1的时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2,  ENABLE);   //使能CAN2时钟,42MHz

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2);

    gpio.GPIO_Pin   = GPIO_Pin_12 | GPIO_Pin_13;
    gpio.GPIO_Mode  = GPIO_Mode_AF; //复用模式
    GPIO_Init(GPIOB, &gpio);        //根据设定参数初始化GPIOB

    nvic.NVIC_IRQChannel                   = CAN2_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;         //抢占优先级   1101
    nvic.NVIC_IRQChannelSubPriority        = 4;         //子优先级
    nvic.NVIC_IRQChannelCmd                = ENABLE;    //IRQ通道使能
    NVIC_Init(&nvic);   //根据指定的参数初始化NVIC寄存器

    /****************************************CAN单元初始化*****************************************/
    CAN_DeInit(CAN2);
    CAN_StructInit(&can2);
    /*CAN主控制寄存器器（CAN_MCR）*/
    can2.CAN_TTCM = DISABLE;//时间触发通信模式
    can2.CAN_ABOM = DISABLE;//自动的总线关闭管理
    can2.CAN_AWUM = DISABLE;//自动唤醒模式
    can2.CAN_NART = DISABLE;//禁止自动重发送
    can2.CAN_RFLM = DISABLE;//接收FIFO锁定模式
    can2.CAN_TXFP = ENABLE; //发送FIFIO优先级
    /*CAN位时序寄存器（CAN_BTR）*/
    /*CAN1波特率=42MHz/3/(9+4+1)=1MHz*/
    can2.CAN_Mode = CAN_Mode_Normal;//普通模式
    can2.CAN_SJW  = CAN_SJW_1tq;    //重新同步跳跃宽度
    can2.CAN_BS1  = CAN_BS1_9tq;    //时间段1
    can2.CAN_BS2  = CAN_BS2_4tq;    //时间段2
    can2.CAN_Prescaler = 3;         //分频系数
    CAN_Init(CAN2, &can2);          //根据指定的参数初始化CAN寄存器
    /*CAN过滤器寄存器*/
    can2_filter.CAN_FilterNumber         = 14;//过滤器14
    can2_filter.CAN_FilterMode           = CAN_FilterMode_IdMask;   //标识符过滤器为掩码模式
    can2_filter.CAN_FilterScale          = CAN_FilterScale_32bit;   //一个32位标识符过滤器
    can2_filter.CAN_FilterIdHigh         = 0x0000;                  //32位标识符过滤器的高16位
    can2_filter.CAN_FilterIdLow          = 0x0000;                  //32位标识符过滤器的低16位
    can2_filter.CAN_FilterMaskIdHigh     = 0x0000;                  //过滤器掩码高16位
    can2_filter.CAN_FilterMaskIdLow      = 0x0000;                  //过滤器掩码低16位
    can2_filter.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;        //过滤器0关联到FIFO0
    can2_filter.CAN_FilterActivation     = ENABLE;                  //激活过滤器
    CAN_FilterInit(&can2_filter);//根据指定的参数初始化CAN_Filter寄存器

    /*CAN中断使能寄存器（CAN_IER）*/
    CAN_ITConfig(CAN2,CAN_IT_FMP0, ENABLE); //接收中断:FIFO 0消息挂起中断
}
