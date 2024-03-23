
#include "usart.h"


/*------------------------------------------------------------------------------------------
串口			|	USART1		USART2		USART3		UART4		UART5		USART6
--------------------------------------------------------------------------------------------
Rx				|	PA10		PD6			PD9			PC11		PD2			PC7
是否使用Rx		|	暂无        暂无		是			暂无		暂无        暂无
Dx_Sx_Cx		|	D2_S2_C4	D1_S5_C4	D1_S1_C4	D1_S2_C4	D1_S0_C4	D2_S1_C5
--------------------------------------------------------------------------------------------
Tx				|	无			PD5			PD8			PC10		PC12		PC6
是否使用Tx		|	暂无        暂无		是			暂无		暂无		暂无
Dx_Sx_Cx		|	D2_S7_C4	D1_S6_C4	D1_S3_C4	D1_S4_C4	D1_S7_C4	D2_S6_C5
--------------------------------------------------------------------------------------------
波特率			|	115200		115200		115200		4096000		115200		115200
--------------------------------------------------------------------------------------------
发送中断机制		|	无			无			串DMA/串							串DMA
接收中断机制		|	DMA+串空	DMA+串空	DMA+串空	        				DMA+串空
中断服务函数		|	U1_I		U2_I	    U3_I		U4_I        U5_I        U6_I
--------------------------------------------------------------------------------------------
功能			|	暂无        暂无        主控通信		暂无        暂无      	暂无
------------------------------------------------------------------------------------------*/


//串口6 调试使用 DMA发送
char UART4_DMA_Buf[1024] = {0};
u32 UART4_DMA_Len = 0;
void UART4_DMA_printf(const char* fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    UART4_DMA_Len = (u32)vsprintf(UART4_DMA_Buf, fmt, ap);   //将格式化数据打到UART4_DMA_Buf字符串上，返回格式化长度

//    while(DMA_GetCurrDataCounter(DMA1_Stream4));        //等之前的发完
    DMA_ClearITPendingBit(DMA1_Stream4, DMA_IT_TCIF4);  //开启DMA_Mode_Normal,即便没有使用完成中断也要软件清除，否则只发一次

    DMA_Cmd(DMA1_Stream4, DISABLE);				        //设置当前计数值前先禁用DMA
    DMA1_Stream4->M0AR = (uint32_t)UART4_DMA_Buf;       //设置当前待发数据基地址:Memory0 tARget
    DMA1_Stream4->NDTR = (uint32_t)UART4_DMA_Len;       //设置当前待发的数据的数量:Number of Data units to be TRansferred
    DMA_Cmd(DMA1_Stream4, ENABLE);				        //启用串口DMA发送
}

/*--------------------------------------------------------------------------
函数功能：无
--------------------------------------------------------------------------*/
void USART1_Configuration(void)
{
    GPIO_InitTypeDef	gpio;
    USART_InitTypeDef	usart;
    NVIC_InitTypeDef 	nvic;
    DMA_InitTypeDef		dma;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA2, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1); 	//TX端口复用
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1); 	//RX端口复用

    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd 	= GPIO_PuPd_UP;
    gpio.GPIO_Mode 	= GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_Pin 	= GPIO_Pin_6 | GPIO_Pin_7; 
    GPIO_Init(GPIOB, &gpio); 

    USART_DeInit(USART1);
    usart.USART_BaudRate			= 115200;       //根据使用情况自定义
    usart.USART_WordLength		    = USART_WordLength_8b;
    usart.USART_StopBits			= USART_StopBits_1;
    usart.USART_Parity				= USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode			  	= USART_Mode_Rx | USART_Mode_Tx;    //接收发送
    USART_Init(USART1, &usart);

   	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);	    //开启空闲中断
    USART_Cmd(USART1, ENABLE);                          //使能串口
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);	    //开启串口DMA接收功能
    
    nvic.NVIC_IRQChannel                     = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority   = 0;
    nvic.NVIC_IRQChannelSubPriority          = 1;
    nvic.NVIC_IRQChannelCmd                  = ENABLE;
    NVIC_Init(&nvic);

    //USART1_Rx
    DMA_DeInit(DMA2_Stream2);
    while( DMA_GetCmdStatus(DMA2_Stream2) == ENABLE );			//等待DMA可配置
    
    dma.DMA_Channel                 =    DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr      =    (uint32_t)&(USART1->DR);		//设置DMA传输外设基地址
    dma.DMA_Memory0BaseAddr         =    (uint32_t)UA1RxDMAbuf;        //设置DMA传输内存基地址
    dma.DMA_DIR                     =    DMA_DIR_PeripheralToMemory;	//设置数据传输方向
    dma.DMA_BufferSize              =    UA1RxDMAbuf_LEN;               //设置DMA一次传输数据量的大小,DR16每隔7ms通过DBus发送一帧数据（18字节）
    dma.DMA_PeripheralInc           =    DMA_PeripheralInc_Disable;		//设置外设地址不变
    dma.DMA_MemoryInc               =    DMA_MemoryInc_Enable;			//设置内存地址递增
    dma.DMA_PeripheralDataSize      =    DMA_PeripheralDataSize_Byte;	//设置外设的数据长度为字节（8bits）
    dma.DMA_MemoryDataSize          =    DMA_MemoryDataSize_Byte;		//设置内存的数据长度为字节（8bits）
    dma.DMA_Mode                    =    DMA_Mode_Circular;				//设置DMA模式为循环模式
    dma.DMA_Priority                =    DMA_Priority_VeryHigh;			//设置DMA通道的优先级为最高优先级
    dma.DMA_FIFOMode                =    DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold           =    DMA_FIFOThreshold_1QuarterFull;
    dma.DMA_MemoryBurst             =    DMA_MemoryBurst_Single;
    dma.DMA_PeripheralBurst         =    DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream2, &dma);
    DMA_Cmd(DMA2_Stream2, ENABLE);	//使能DMA
}
/*--------------------------------------------------------------------------
函数功能：给视觉发送四元数姿态以及角速度、线加速度信息
--------------------------------------------------------------------------*/

void USART2_Configuration(void)
{
    GPIO_InitTypeDef	gpio;
    USART_InitTypeDef	usart;
    NVIC_InitTypeDef 	nvic;
    DMA_InitTypeDef		dma;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_DMA1, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2); 	//TX端口复用
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2); 	//RX端口复用

    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd 	= GPIO_PuPd_UP;
    gpio.GPIO_Mode 	= GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_Pin 	= GPIO_Pin_2 | GPIO_Pin_3; 
    GPIO_Init(GPIOA, &gpio); 

    USART_DeInit(USART1);
    usart.USART_BaudRate			= 115200;       //根据使用情况自定义
    usart.USART_WordLength		    = USART_WordLength_8b;
    usart.USART_StopBits			= USART_StopBits_1;
    usart.USART_Parity				= USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode			  	= USART_Mode_Rx | USART_Mode_Tx;    //接收发送
    USART_Init(USART2, &usart);

    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);   //开启空闲中断
    USART_Cmd(USART2, ENABLE);                       //使能串口
    USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);   //使能串口发送DMA
    USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);   //使能串口接收DMA

    nvic.NVIC_IRQChannel					= USART2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority  = 0;
    nvic.NVIC_IRQChannelSubPriority			= 10;
    nvic.NVIC_IRQChannelCmd					= ENABLE;
    NVIC_Init(&nvic);

    //Tx
    DMA_DeInit(DMA1_Stream6);
    while( DMA_GetCmdStatus(DMA1_Stream6) == ENABLE );			//等待DMA可配置

    dma.DMA_Channel			      	=   DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr	    =	(uint32_t)&(USART2->DR);
    dma.DMA_Memory0BaseAddr	  	    =	NULL;//暂无
    dma.DMA_DIR				        =	DMA_DIR_MemoryToPeripheral;	//内存到外设
    dma.DMA_BufferSize	    		=	NULL;//暂无
    dma.DMA_PeripheralInc		    =	DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc		      	=	DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize	    =	DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize	    	=	DMA_MemoryDataSize_Byte;
    dma.DMA_Mode                    =	DMA_Mode_Normal;			//正常发送
    dma.DMA_Priority	      		=	DMA_Priority_VeryHigh;
    dma.DMA_FIFOMode		      	=	DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold	    	=	DMA_FIFOThreshold_1QuarterFull;
    dma.DMA_MemoryBurst		    	=	DMA_MemoryBurst_Single;
    dma.DMA_PeripheralBurst	    	=	DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream6, &dma);
    DMA_Cmd(DMA1_Stream6, ENABLE);	//使能DMA

    //RX
    DMA_DeInit(DMA1_Stream5);
    while( DMA_GetCmdStatus(DMA1_Stream5) == ENABLE );			//等待DMA可配置

    dma.DMA_Channel            =     DMA_Channel_4;				//即DMA_Channel_5
    dma.DMA_PeripheralBaseAddr = 	(uint32_t)&(USART2->DR);
    dma.DMA_Memory0BaseAddr    = 	(uint32_t)UA2RxDMAbuf;
    dma.DMA_DIR                = 	DMA_DIR_PeripheralToMemory;	//外设到内存
    dma.DMA_BufferSize         = 	UA2RxDMAbuf_LEN;            //接收数组长度，自定义
    dma.DMA_Mode               = 	DMA_Mode_Circular;			//循环接收
    dma.DMA_Priority           = 	DMA_Priority_VeryHigh;
    DMA_Init(DMA1_Stream5, &dma);
    DMA_Cmd(DMA1_Stream5, ENABLE);	//使能DMA--暂时用不到收
}
/*--------------------------------------------------------------------------
函数功能：无
--------------------------------------------------------------------------*/
void USART3_Configuration(void)
{
    GPIO_InitTypeDef	gpio;
    USART_InitTypeDef	usart;
    NVIC_InitTypeDef 	nvic;
    DMA_InitTypeDef		dma;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA1, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3); 	//TX端口复用
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3); 	//RX端口复用

    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd 	= GPIO_PuPd_UP;
    gpio.GPIO_Mode 	= GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_Pin 	= GPIO_Pin_10 | GPIO_Pin_11; 
    GPIO_Init(GPIOB, &gpio); 

    USART_DeInit(USART1);
    usart.USART_BaudRate			= 115200;       //根据使用情况自定义
    usart.USART_WordLength		    = USART_WordLength_8b;
    usart.USART_StopBits			= USART_StopBits_1;
    usart.USART_Parity				= USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode			  	= USART_Mode_Rx | USART_Mode_Tx;    //接收发送
    USART_Init(USART3, &usart);

    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);   //开启空闲中断
    USART_Cmd(USART3, ENABLE);                       //使能串口
    USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);   //使能串口发送DMA
    USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);   //使能串口接收DMA

    nvic.NVIC_IRQChannel					= USART3_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority  = 0;
    nvic.NVIC_IRQChannelSubPriority			= 10;
    nvic.NVIC_IRQChannelCmd					= ENABLE;
    NVIC_Init(&nvic);

    //Tx
    DMA_DeInit(DMA1_Stream3);
    while( DMA_GetCmdStatus(DMA1_Stream3) == ENABLE );			//等待DMA可配置

    dma.DMA_Channel			      	=   DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr	    =	(uint32_t)&(USART3->DR);
    dma.DMA_Memory0BaseAddr	  	    =	NULL;//暂无
    dma.DMA_DIR				        =	DMA_DIR_MemoryToPeripheral;	//内存到外设
    dma.DMA_BufferSize	    		=	NULL;//暂无
    dma.DMA_PeripheralInc		    =	DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc		      	=	DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize	    =	DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize	    	=	DMA_MemoryDataSize_Byte;
    dma.DMA_Mode                    =	DMA_Mode_Normal;			//正常发送
    dma.DMA_Priority	      		=	DMA_Priority_VeryHigh;
    dma.DMA_FIFOMode		      	=	DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold	    	=	DMA_FIFOThreshold_1QuarterFull;
    dma.DMA_MemoryBurst		    	=	DMA_MemoryBurst_Single;
    dma.DMA_PeripheralBurst	    	=	DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream3, &dma);
    DMA_Cmd(DMA1_Stream3, DISABLE);	//失能DMA

    //RX
    DMA_DeInit(DMA1_Stream1);
    while( DMA_GetCmdStatus(DMA1_Stream1) == ENABLE );			//等待DMA可配置

    dma.DMA_Channel            =     DMA_Channel_4;				//即DMA_Channel_5
    dma.DMA_PeripheralBaseAddr = 	(uint32_t)&(USART3->DR);
    dma.DMA_Memory0BaseAddr    = 	(uint32_t)UA3RxDMAbuf;
    dma.DMA_DIR                = 	DMA_DIR_PeripheralToMemory;	//外设到内存
    dma.DMA_BufferSize         = 	UA3RxDMAbuf_LEN;            //接收数组长度，自定义
    dma.DMA_Mode               = 	DMA_Mode_Circular;			//循环接收
    dma.DMA_Priority           = 	DMA_Priority_VeryHigh;
    DMA_Init(DMA1_Stream1, &dma);
    DMA_Cmd(DMA1_Stream1, ENABLE);	//使能DMA
}
/*--------------------------------------------------------------------------
函数功能：主控通信
--------------------------------------------------------------------------*/
extern u8 UA4RxDMAbuf[];
void UART4_Configuration(void)
{
    GPIO_InitTypeDef	gpio= {0};
    USART_InitTypeDef	usart= {0};
    NVIC_InitTypeDef 	nvic= {0};
    DMA_InitTypeDef		dma= {0};

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);						//使能UART4时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_DMA1,ENABLE);	//使能PC端口时钟

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);//Tx
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4);//Rx

    gpio.GPIO_OType = GPIO_OType_PP;//PP
    gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOA, &gpio);

    nvic.NVIC_IRQChannel 					= UART4_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority	= 0;		//抢占优先级
    nvic.NVIC_IRQChannelSubPriority			= 6;		//子优先级
    nvic.NVIC_IRQChannelCmd					= ENABLE;	//IRQ通道使能
    NVIC_Init(&nvic);//根据指定的参数初始化NVIC寄存器

    usart.USART_BaudRate			= 460800;			  	//波特率
    usart.USART_WordLength			= USART_WordLength_8b;	//字长为8位数据格式
    usart.USART_StopBits			= USART_StopBits_1;		//一个停止位
    usart.USART_Parity				= USART_Parity_No;		//无奇偶校验位
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//无硬件数据流控制
    usart.USART_Mode				= USART_Mode_Tx|USART_Mode_Rx;		//收发模式
    USART_Init(UART4, &usart);  //初始化串口

    USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);	//开启空闲中断
    USART_Cmd(UART4, ENABLE);	//使能串口
    USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);
    USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE);

    //UART4_Tx
    DMA_DeInit(DMA1_Stream4);
    while( DMA_GetCmdStatus(DMA1_Stream4) == ENABLE );			//等待DMA可配置

    dma.DMA_Channel				=	DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr	=	(uint32_t)&(UART4->DR);
    dma.DMA_Memory0BaseAddr		=	NULL;//暂无
    dma.DMA_DIR					=	DMA_DIR_MemoryToPeripheral;	//内存到外设
    dma.DMA_BufferSize			=	NULL;//暂无
    dma.DMA_PeripheralInc		=	DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc			=	DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize	=	DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize		=	DMA_MemoryDataSize_Byte;
    dma.DMA_Mode				=	DMA_Mode_Normal;			//正常发送
    dma.DMA_Priority			=	DMA_Priority_VeryHigh;
    dma.DMA_FIFOMode			=	DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold		=	DMA_FIFOThreshold_1QuarterFull;
    dma.DMA_MemoryBurst			=	DMA_MemoryBurst_Single;
    dma.DMA_PeripheralBurst		=	DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream4, &dma);
    DMA_Cmd(DMA1_Stream4, DISABLE);

    //UART4_Rx
    DMA_DeInit(DMA1_Stream2);
    while( DMA_GetCmdStatus(DMA1_Stream2) == ENABLE );			//等待DMA可配置

    dma.DMA_Channel            = 	DMA_Channel_4;				//即DMA_Channel_4
    dma.DMA_PeripheralBaseAddr = 	(uint32_t)&(UART4->DR);
    dma.DMA_Memory0BaseAddr    = 	(uint32_t)UA4RxDMAbuf;
    dma.DMA_DIR                = 	DMA_DIR_PeripheralToMemory;	//外设到内存
    dma.DMA_BufferSize         = 	UA4RxDMAbuf_LEN;
    dma.DMA_Mode               = 	DMA_Mode_Circular;			//循环接收
    dma.DMA_Priority           = 	DMA_Priority_VeryHigh;
    DMA_Init(DMA1_Stream2, &dma);
    DMA_Cmd(DMA1_Stream2, ENABLE);
}
/*--------------------------------------------------------------------------
函数功能：无
--------------------------------------------------------------------------*/
void USART6_Configuration(void)
{
    GPIO_InitTypeDef	gpio;
    USART_InitTypeDef	usart;
    NVIC_InitTypeDef 	nvic;
    DMA_InitTypeDef		dma;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_DMA2, ENABLE);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6); 	//TX端口复用
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6); 	//RX端口复用

    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd 	= GPIO_PuPd_UP;
    gpio.GPIO_Mode 	= GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_Pin 	= GPIO_Pin_6 | GPIO_Pin_7; 
    GPIO_Init(GPIOC, &gpio); 

    USART_DeInit(USART1);
    usart.USART_BaudRate			= 115200;       //根据使用情况自定义
    usart.USART_WordLength		    = USART_WordLength_8b;
    usart.USART_StopBits			= USART_StopBits_1;
    usart.USART_Parity				= USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode			  	= USART_Mode_Rx | USART_Mode_Tx;    //接收发送
    USART_Init(USART6, &usart);

    USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);   //开启空闲中断
    USART_Cmd(USART6, ENABLE);                       //使能串口
    USART_DMACmd(USART6, USART_DMAReq_Tx, ENABLE);   //使能串口发送DMA
    USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);   //使能串口接收DMA

    nvic.NVIC_IRQChannel					= USART6_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority  = 0;
    nvic.NVIC_IRQChannelSubPriority			= 10;
    nvic.NVIC_IRQChannelCmd					= ENABLE;
    NVIC_Init(&nvic);

    //Tx
    DMA_DeInit(DMA2_Stream6);
    while( DMA_GetCmdStatus(DMA2_Stream6) == ENABLE );			//等待DMA可配置

    dma.DMA_Channel			      	=   DMA_Channel_5;
    dma.DMA_PeripheralBaseAddr	    =	(uint32_t)&(USART6->DR);
    dma.DMA_Memory0BaseAddr	  	    =	NULL;//暂无
    dma.DMA_DIR				        =	DMA_DIR_MemoryToPeripheral;	//内存到外设
    dma.DMA_BufferSize	    		=	NULL;//暂无
    dma.DMA_PeripheralInc		    =	DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc		      	=	DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize	    =	DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize	    	=	DMA_MemoryDataSize_Byte;
    dma.DMA_Mode                    =	DMA_Mode_Normal;			//正常发送
    dma.DMA_Priority	      		=	DMA_Priority_VeryHigh;
    dma.DMA_FIFOMode		      	=	DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold	    	=	DMA_FIFOThreshold_1QuarterFull;
    dma.DMA_MemoryBurst		    	=	DMA_MemoryBurst_Single;
    dma.DMA_PeripheralBurst	    	=	DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream6, &dma);
    DMA_Cmd(DMA2_Stream6, DISABLE);	//失能DMA

    //RX
    DMA_DeInit(DMA2_Stream1);
    while( DMA_GetCmdStatus(DMA2_Stream1) == ENABLE );			//等待DMA可配置

    dma.DMA_Channel            =   DMA_Channel_5;				//即DMA_Channel_5
    dma.DMA_PeripheralBaseAddr = 	(uint32_t)&(USART6->DR);
    dma.DMA_Memory0BaseAddr    = 	(uint32_t)UA6RxDMAbuf;
    dma.DMA_DIR                = 	DMA_DIR_PeripheralToMemory;	//外设到内存
    dma.DMA_BufferSize         = 	UA6RxDMAbuf_LEN;            //接收数组长度，自定义
    dma.DMA_Mode               = 	DMA_Mode_Circular;			//循环接收
    dma.DMA_Priority           = 	DMA_Priority_VeryHigh;
    DMA_Init(DMA2_Stream1, &dma);
    DMA_Cmd(DMA2_Stream1, ENABLE);	//使能DMA
}
