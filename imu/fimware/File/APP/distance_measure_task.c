#include "distance_measure_task.h"

char DistanceMeasureUA1Buf[10] = {0};
char DistanceMeasureUA2Buf[10] = {0};
char DistanceMeasureUA3Buf[10] = {0};

float X_Axis_Distance = 0.0f;
float Y_Axis_Distance = 0.0f;
float Z_Axis_Distance = 0.0f;

/*--------------------------------------------------------------------------
�������ܣ�����1�ײ����ã�X����ͨѶ��
--------------------------------------------------------------------------*/
void UA1_Configuration(void)
{
    GPIO_InitTypeDef	gpio;
    USART_InitTypeDef	usart;
    NVIC_InitTypeDef 	nvic;
    DMA_InitTypeDef		dma;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1); 	//TX
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10,  GPIO_AF_USART1); //RX

    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd 	= GPIO_PuPd_NOPULL;
    gpio.GPIO_Mode 	= GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_Pin 	= GPIO_Pin_9; 	//TX
    GPIO_Init(GPIOA, &gpio);
    gpio.GPIO_Pin 	= GPIO_Pin_10;
    GPIO_Init(GPIOA, &gpio); 	    //RX

    USART_DeInit(USART1);
    usart.USART_BaudRate			= 9600;//�Ӿ�����
    usart.USART_WordLength		= USART_WordLength_8b;
    usart.USART_StopBits			= USART_StopBits_1;
    usart.USART_Parity				= USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode			  	= USART_Mode_Rx | USART_Mode_Tx;//�����ж�
    USART_Init(USART1, &usart);

    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);   //���������ж�
    USART_Cmd(USART1, ENABLE);                       //ʹ�ܴ���
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);   //ʹ�ܴ��ڷ���DMA
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);   //ʹ�ܴ��ڽ���DMA

    nvic.NVIC_IRQChannel					= USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority  = 0;
    nvic.NVIC_IRQChannelSubPriority			= 10;
    nvic.NVIC_IRQChannelCmd					= ENABLE;
    NVIC_Init(&nvic);

    //Tx
    DMA_DeInit(DMA2_Stream7);
    while( DMA_GetCmdStatus(DMA2_Stream7) == ENABLE );			//�ȴ�DMA������

    dma.DMA_Channel			      	=   DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr	=	(uint32_t)&(USART1->DR);
    dma.DMA_Memory0BaseAddr	  	=	NULL;//����
    dma.DMA_DIR				        	=	DMA_DIR_MemoryToPeripheral;	//�ڴ浽����
    dma.DMA_BufferSize	    		=	NULL;//����
    dma.DMA_PeripheralInc		    =	DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc		      	=	DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize	=	DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize	  	=	DMA_MemoryDataSize_Byte;
    dma.DMA_Mode			        	=	DMA_Mode_Normal;			//��������
    dma.DMA_Priority	      		=	DMA_Priority_VeryHigh;
    dma.DMA_FIFOMode		      	=	DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold	    	=	DMA_FIFOThreshold_1QuarterFull;
    dma.DMA_MemoryBurst		    	=	DMA_MemoryBurst_Single;
    dma.DMA_PeripheralBurst	   	=	DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream7, &dma);
    DMA_Cmd(DMA2_Stream7, DISABLE);	//ʧ��DMA

    //RX
    DMA_DeInit(DMA2_Stream2);
    while( DMA_GetCmdStatus(DMA2_Stream2) == ENABLE );			//�ȴ�DMA������

    dma.DMA_Channel            =  DMA_Channel_4;				//��DMA_Channel_5
    dma.DMA_PeripheralBaseAddr = 	(uint32_t)&(USART1->DR);
    dma.DMA_Memory0BaseAddr    = 	(uint32_t)DistanceMeasureUA1Buf;
    dma.DMA_DIR                = 	DMA_DIR_PeripheralToMemory;	//���赽�ڴ�
    dma.DMA_BufferSize         = 	10;
    dma.DMA_Mode               = 	DMA_Mode_Circular;			//ѭ������
    dma.DMA_Priority           = 	DMA_Priority_VeryHigh;
    DMA_Init(DMA2_Stream2, &dma);
    DMA_Cmd(DMA2_Stream2, ENABLE);	//ʹ��DMA
}


/*--------------------------------------------------------------------------
�������ܣ�����2�ײ����ã�Y����ͨѶ��
--------------------------------------------------------------------------*/
void UA2_Configuration(void)
{
    GPIO_InitTypeDef	gpio;
    USART_InitTypeDef	usart;
    NVIC_InitTypeDef 	nvic;
    DMA_InitTypeDef		dma;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_DMA1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2); 	//TX
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3,  GPIO_AF_USART2); 	//RX

    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd 	= GPIO_PuPd_NOPULL;
    gpio.GPIO_Mode 	= GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_Pin 	= GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_Init(GPIOA, &gpio);

    USART_DeInit(USART2);
    usart.USART_BaudRate			= 9600;//���ģ��
    usart.USART_WordLength		= USART_WordLength_8b;
    usart.USART_StopBits			= USART_StopBits_1;
    usart.USART_Parity				= USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode			  	= USART_Mode_Rx | USART_Mode_Tx;//�����ж�
    USART_Init(USART2, &usart);

    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);   //���������ж�
    USART_Cmd(USART2, ENABLE);                       //ʹ�ܴ���
    USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);   //ʹ�ܴ��ڷ���DMA
    USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);   //ʹ�ܴ��ڽ���DMA

    nvic.NVIC_IRQChannel				          	= USART2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority  = 0;
    nvic.NVIC_IRQChannelSubPriority		    	= 9;
    nvic.NVIC_IRQChannelCmd				        	= ENABLE;
    NVIC_Init(&nvic);

    //Tx
    DMA_DeInit(DMA1_Stream6);
    while( DMA_GetCmdStatus(DMA1_Stream6) == ENABLE );			//�ȴ�DMA������

    dma.DMA_Channel			       	= DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr	=	(uint32_t)&(USART2->DR);
    dma.DMA_Memory0BaseAddr	  	=	NULL;//����
    dma.DMA_DIR				        	=	DMA_DIR_MemoryToPeripheral;	//�ڴ浽����
    dma.DMA_BufferSize		    	=	NULL;//����
    dma.DMA_PeripheralInc	      =	DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc		      	=	DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize	=	DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize	  	=	DMA_MemoryDataSize_Byte;
    dma.DMA_Mode			        	=	DMA_Mode_Normal;			//��������
    dma.DMA_Priority	      		=	DMA_Priority_VeryHigh;
    dma.DMA_FIFOMode		      	=	DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold	    	=	DMA_FIFOThreshold_1QuarterFull;
    dma.DMA_MemoryBurst		    	=	DMA_MemoryBurst_Single;
    dma.DMA_PeripheralBurst	  	=	DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream6, &dma);
    DMA_Cmd(DMA1_Stream6, DISABLE);	//ʧ��DMA

    //RX
    DMA_DeInit(DMA1_Stream5);
    while( DMA_GetCmdStatus(DMA1_Stream5) == ENABLE );			//�ȴ�DMA������

    dma.DMA_Channel            =  DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr = 	(uint32_t)&(USART2->DR);
    dma.DMA_Memory0BaseAddr    = 	(uint32_t)DistanceMeasureUA2Buf;
    dma.DMA_DIR                = 	DMA_DIR_PeripheralToMemory;	//���赽�ڴ�
    dma.DMA_BufferSize         = 	10;
    dma.DMA_Mode               = 	DMA_Mode_Circular;			//ѭ������
    dma.DMA_Priority           = 	DMA_Priority_VeryHigh;
    DMA_Init(DMA1_Stream5, &dma);
    DMA_Cmd(DMA1_Stream5, ENABLE);	//ʹ��DMA
}


/*--------------------------------------------------------------------------
�������ܣ�����3�ײ�����
��    ע��Z����ͨѶ��
--------------------------------------------------------------------------*/
void UA3_Configuration(void)
{
    GPIO_InitTypeDef	gpio;
    USART_InitTypeDef	usart;
    NVIC_InitTypeDef 	nvic;
    DMA_InitTypeDef		dma;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3); //TX
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3); //RX

    gpio.GPIO_OType	=	GPIO_OType_PP;//PP
    gpio.GPIO_PuPd	=	GPIO_PuPd_UP;
    gpio.GPIO_Mode	=	GPIO_Mode_AF;
    gpio.GPIO_Pin	=	GPIO_Pin_10 | GPIO_Pin_11;
    gpio.GPIO_Speed	=	GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    USART_DeInit(USART3);
    usart.USART_BaudRate			=	9600;//���ģ��
    usart.USART_WordLength		=	USART_WordLength_8b;
    usart.USART_StopBits			=	USART_StopBits_1;
    usart.USART_Parity				=	USART_Parity_No;
    usart.USART_HardwareFlowControl	=	USART_HardwareFlowControl_None;
    usart.USART_Mode			  	=	USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART3, &usart);

    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);		//����3���տ����ж�
    USART_Cmd(USART3, ENABLE);
    USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
    USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);

    nvic.NVIC_IRQChannel					=	USART3_IRQn; 	//����3���տ����ж�
    nvic.NVIC_IRQChannelPreemptionPriority	=	0;
    nvic.NVIC_IRQChannelSubPriority			=	10;
    nvic.NVIC_IRQChannelCmd					=	ENABLE;
    NVIC_Init(&nvic);

    //USART3_Tx
    DMA_DeInit(DMA1_Stream3);
    while( DMA_GetCmdStatus(DMA1_Stream3) == ENABLE );			//�ȴ�DMA������

    dma.DMA_Channel				      =	DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr	=	(uint32_t)&(USART3->DR);
    dma.DMA_Memory0BaseAddr	  	=	NULL;//����
    dma.DMA_DIR				        	=	DMA_DIR_MemoryToPeripheral;	//�ڴ浽����
    dma.DMA_BufferSize		    	=	NULL;//����
    dma.DMA_PeripheralInc	    	=	DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc		      	=	DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize	=	DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize	  	=	DMA_MemoryDataSize_Byte;
    dma.DMA_Mode			        	=	DMA_Mode_Normal;			//��������
    dma.DMA_Priority		      	=	DMA_Priority_VeryHigh;
    dma.DMA_FIFOMode		      	=	DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold	    	=	DMA_FIFOThreshold_1QuarterFull;
    dma.DMA_MemoryBurst		    	=	DMA_MemoryBurst_Single;
    dma.DMA_PeripheralBurst	   	=	DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream3, &dma);
    DMA_Cmd(DMA1_Stream3, DISABLE);

    //USART3_Rx
    DMA_DeInit(DMA1_Stream1);
    while( DMA_GetCmdStatus(DMA1_Stream1) == ENABLE );			//�ȴ�DMA������

    dma.DMA_Channel            = 	DMA_Channel_4;				//��DMA_Channel_4
    dma.DMA_PeripheralBaseAddr = 	(uint32_t)&(USART3->DR);
    dma.DMA_Memory0BaseAddr    = 	(uint32_t)DistanceMeasureUA3Buf;
    dma.DMA_DIR                = 	DMA_DIR_PeripheralToMemory;	//���赽�ڴ�
    dma.DMA_BufferSize         = 	10;
    dma.DMA_Mode               = 	DMA_Mode_Circular;			//ѭ������
    dma.DMA_Priority           = 	DMA_Priority_VeryHigh;
    DMA_Init(DMA1_Stream1, &dma);
    DMA_Cmd(DMA1_Stream1, ENABLE);
}
