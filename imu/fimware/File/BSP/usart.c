
#include "usart.h"


/*------------------------------------------------------------------------------------------
����			|	USART1		USART2		USART3		UART4		UART5		USART6
--------------------------------------------------------------------------------------------
Rx				|	PA10		PD6			PD9			PC11		PD2			PC7
�Ƿ�ʹ��Rx		|	����        ����		��			����		����        ����
Dx_Sx_Cx		|	D2_S2_C4	D1_S5_C4	D1_S1_C4	D1_S2_C4	D1_S0_C4	D2_S1_C5
--------------------------------------------------------------------------------------------
Tx				|	��			PD5			PD8			PC10		PC12		PC6
�Ƿ�ʹ��Tx		|	����        ����		��			����		����		����
Dx_Sx_Cx		|	D2_S7_C4	D1_S6_C4	D1_S3_C4	D1_S4_C4	D1_S7_C4	D2_S6_C5
--------------------------------------------------------------------------------------------
������			|	115200		115200		115200		4096000		115200		115200
--------------------------------------------------------------------------------------------
�����жϻ���		|	��			��			��DMA/��							��DMA
�����жϻ���		|	DMA+����	DMA+����	DMA+����	        				DMA+����
�жϷ�����		|	U1_I		U2_I	    U3_I		U4_I        U5_I        U6_I
--------------------------------------------------------------------------------------------
����			|	����        ����        ����ͨ��		����        ����      	����
------------------------------------------------------------------------------------------*/


//����6 ����ʹ�� DMA����
char UART4_DMA_Buf[1024] = {0};
u32 UART4_DMA_Len = 0;
void UART4_DMA_printf(const char* fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    UART4_DMA_Len = (u32)vsprintf(UART4_DMA_Buf, fmt, ap);   //����ʽ�����ݴ�UART4_DMA_Buf�ַ����ϣ����ظ�ʽ������

//    while(DMA_GetCurrDataCounter(DMA1_Stream4));        //��֮ǰ�ķ���
    DMA_ClearITPendingBit(DMA1_Stream4, DMA_IT_TCIF4);  //����DMA_Mode_Normal,����û��ʹ������ж�ҲҪ������������ֻ��һ��

    DMA_Cmd(DMA1_Stream4, DISABLE);				        //���õ�ǰ����ֵǰ�Ƚ���DMA
    DMA1_Stream4->M0AR = (uint32_t)UART4_DMA_Buf;       //���õ�ǰ�������ݻ���ַ:Memory0 tARget
    DMA1_Stream4->NDTR = (uint32_t)UART4_DMA_Len;       //���õ�ǰ���������ݵ�����:Number of Data units to be TRansferred
    DMA_Cmd(DMA1_Stream4, ENABLE);				        //���ô���DMA����
}

/*--------------------------------------------------------------------------
�������ܣ���
--------------------------------------------------------------------------*/
void USART1_Configuration(void)
{
    GPIO_InitTypeDef	gpio;
    USART_InitTypeDef	usart;
    NVIC_InitTypeDef 	nvic;
    DMA_InitTypeDef		dma;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA2, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1); 	//TX�˿ڸ���
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1); 	//RX�˿ڸ���

    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd 	= GPIO_PuPd_UP;
    gpio.GPIO_Mode 	= GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_Pin 	= GPIO_Pin_6 | GPIO_Pin_7; 
    GPIO_Init(GPIOB, &gpio); 

    USART_DeInit(USART1);
    usart.USART_BaudRate			= 115200;       //����ʹ������Զ���
    usart.USART_WordLength		    = USART_WordLength_8b;
    usart.USART_StopBits			= USART_StopBits_1;
    usart.USART_Parity				= USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode			  	= USART_Mode_Rx | USART_Mode_Tx;    //���շ���
    USART_Init(USART1, &usart);

   	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);	    //���������ж�
    USART_Cmd(USART1, ENABLE);                          //ʹ�ܴ���
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);	    //��������DMA���չ���
    
    nvic.NVIC_IRQChannel                     = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority   = 0;
    nvic.NVIC_IRQChannelSubPriority          = 1;
    nvic.NVIC_IRQChannelCmd                  = ENABLE;
    NVIC_Init(&nvic);

    //USART1_Rx
    DMA_DeInit(DMA2_Stream2);
    while( DMA_GetCmdStatus(DMA2_Stream2) == ENABLE );			//�ȴ�DMA������
    
    dma.DMA_Channel                 =    DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr      =    (uint32_t)&(USART1->DR);		//����DMA�����������ַ
    dma.DMA_Memory0BaseAddr         =    (uint32_t)UA1RxDMAbuf;        //����DMA�����ڴ����ַ
    dma.DMA_DIR                     =    DMA_DIR_PeripheralToMemory;	//�������ݴ��䷽��
    dma.DMA_BufferSize              =    UA1RxDMAbuf_LEN;               //����DMAһ�δ����������Ĵ�С,DR16ÿ��7msͨ��DBus����һ֡���ݣ�18�ֽڣ�
    dma.DMA_PeripheralInc           =    DMA_PeripheralInc_Disable;		//���������ַ����
    dma.DMA_MemoryInc               =    DMA_MemoryInc_Enable;			//�����ڴ��ַ����
    dma.DMA_PeripheralDataSize      =    DMA_PeripheralDataSize_Byte;	//������������ݳ���Ϊ�ֽڣ�8bits��
    dma.DMA_MemoryDataSize          =    DMA_MemoryDataSize_Byte;		//�����ڴ�����ݳ���Ϊ�ֽڣ�8bits��
    dma.DMA_Mode                    =    DMA_Mode_Circular;				//����DMAģʽΪѭ��ģʽ
    dma.DMA_Priority                =    DMA_Priority_VeryHigh;			//����DMAͨ�������ȼ�Ϊ������ȼ�
    dma.DMA_FIFOMode                =    DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold           =    DMA_FIFOThreshold_1QuarterFull;
    dma.DMA_MemoryBurst             =    DMA_MemoryBurst_Single;
    dma.DMA_PeripheralBurst         =    DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream2, &dma);
    DMA_Cmd(DMA2_Stream2, ENABLE);	//ʹ��DMA
}
/*--------------------------------------------------------------------------
�������ܣ����Ӿ�������Ԫ����̬�Լ����ٶȡ��߼��ٶ���Ϣ
--------------------------------------------------------------------------*/

void USART2_Configuration(void)
{
    GPIO_InitTypeDef	gpio;
    USART_InitTypeDef	usart;
    NVIC_InitTypeDef 	nvic;
    DMA_InitTypeDef		dma;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_DMA1, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2); 	//TX�˿ڸ���
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2); 	//RX�˿ڸ���

    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd 	= GPIO_PuPd_UP;
    gpio.GPIO_Mode 	= GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_Pin 	= GPIO_Pin_2 | GPIO_Pin_3; 
    GPIO_Init(GPIOA, &gpio); 

    USART_DeInit(USART1);
    usart.USART_BaudRate			= 115200;       //����ʹ������Զ���
    usart.USART_WordLength		    = USART_WordLength_8b;
    usart.USART_StopBits			= USART_StopBits_1;
    usart.USART_Parity				= USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode			  	= USART_Mode_Rx | USART_Mode_Tx;    //���շ���
    USART_Init(USART2, &usart);

    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);   //���������ж�
    USART_Cmd(USART2, ENABLE);                       //ʹ�ܴ���
    USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);   //ʹ�ܴ��ڷ���DMA
    USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);   //ʹ�ܴ��ڽ���DMA

    nvic.NVIC_IRQChannel					= USART2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority  = 0;
    nvic.NVIC_IRQChannelSubPriority			= 10;
    nvic.NVIC_IRQChannelCmd					= ENABLE;
    NVIC_Init(&nvic);

    //Tx
    DMA_DeInit(DMA1_Stream6);
    while( DMA_GetCmdStatus(DMA1_Stream6) == ENABLE );			//�ȴ�DMA������

    dma.DMA_Channel			      	=   DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr	    =	(uint32_t)&(USART2->DR);
    dma.DMA_Memory0BaseAddr	  	    =	NULL;//����
    dma.DMA_DIR				        =	DMA_DIR_MemoryToPeripheral;	//�ڴ浽����
    dma.DMA_BufferSize	    		=	NULL;//����
    dma.DMA_PeripheralInc		    =	DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc		      	=	DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize	    =	DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize	    	=	DMA_MemoryDataSize_Byte;
    dma.DMA_Mode                    =	DMA_Mode_Normal;			//��������
    dma.DMA_Priority	      		=	DMA_Priority_VeryHigh;
    dma.DMA_FIFOMode		      	=	DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold	    	=	DMA_FIFOThreshold_1QuarterFull;
    dma.DMA_MemoryBurst		    	=	DMA_MemoryBurst_Single;
    dma.DMA_PeripheralBurst	    	=	DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream6, &dma);
    DMA_Cmd(DMA1_Stream6, ENABLE);	//ʹ��DMA

    //RX
    DMA_DeInit(DMA1_Stream5);
    while( DMA_GetCmdStatus(DMA1_Stream5) == ENABLE );			//�ȴ�DMA������

    dma.DMA_Channel            =     DMA_Channel_4;				//��DMA_Channel_5
    dma.DMA_PeripheralBaseAddr = 	(uint32_t)&(USART2->DR);
    dma.DMA_Memory0BaseAddr    = 	(uint32_t)UA2RxDMAbuf;
    dma.DMA_DIR                = 	DMA_DIR_PeripheralToMemory;	//���赽�ڴ�
    dma.DMA_BufferSize         = 	UA2RxDMAbuf_LEN;            //�������鳤�ȣ��Զ���
    dma.DMA_Mode               = 	DMA_Mode_Circular;			//ѭ������
    dma.DMA_Priority           = 	DMA_Priority_VeryHigh;
    DMA_Init(DMA1_Stream5, &dma);
    DMA_Cmd(DMA1_Stream5, ENABLE);	//ʹ��DMA--��ʱ�ò�����
}
/*--------------------------------------------------------------------------
�������ܣ���
--------------------------------------------------------------------------*/
void USART3_Configuration(void)
{
    GPIO_InitTypeDef	gpio;
    USART_InitTypeDef	usart;
    NVIC_InitTypeDef 	nvic;
    DMA_InitTypeDef		dma;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA1, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3); 	//TX�˿ڸ���
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3); 	//RX�˿ڸ���

    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd 	= GPIO_PuPd_UP;
    gpio.GPIO_Mode 	= GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_Pin 	= GPIO_Pin_10 | GPIO_Pin_11; 
    GPIO_Init(GPIOB, &gpio); 

    USART_DeInit(USART1);
    usart.USART_BaudRate			= 115200;       //����ʹ������Զ���
    usart.USART_WordLength		    = USART_WordLength_8b;
    usart.USART_StopBits			= USART_StopBits_1;
    usart.USART_Parity				= USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode			  	= USART_Mode_Rx | USART_Mode_Tx;    //���շ���
    USART_Init(USART3, &usart);

    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);   //���������ж�
    USART_Cmd(USART3, ENABLE);                       //ʹ�ܴ���
    USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);   //ʹ�ܴ��ڷ���DMA
    USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);   //ʹ�ܴ��ڽ���DMA

    nvic.NVIC_IRQChannel					= USART3_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority  = 0;
    nvic.NVIC_IRQChannelSubPriority			= 10;
    nvic.NVIC_IRQChannelCmd					= ENABLE;
    NVIC_Init(&nvic);

    //Tx
    DMA_DeInit(DMA1_Stream3);
    while( DMA_GetCmdStatus(DMA1_Stream3) == ENABLE );			//�ȴ�DMA������

    dma.DMA_Channel			      	=   DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr	    =	(uint32_t)&(USART3->DR);
    dma.DMA_Memory0BaseAddr	  	    =	NULL;//����
    dma.DMA_DIR				        =	DMA_DIR_MemoryToPeripheral;	//�ڴ浽����
    dma.DMA_BufferSize	    		=	NULL;//����
    dma.DMA_PeripheralInc		    =	DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc		      	=	DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize	    =	DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize	    	=	DMA_MemoryDataSize_Byte;
    dma.DMA_Mode                    =	DMA_Mode_Normal;			//��������
    dma.DMA_Priority	      		=	DMA_Priority_VeryHigh;
    dma.DMA_FIFOMode		      	=	DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold	    	=	DMA_FIFOThreshold_1QuarterFull;
    dma.DMA_MemoryBurst		    	=	DMA_MemoryBurst_Single;
    dma.DMA_PeripheralBurst	    	=	DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream3, &dma);
    DMA_Cmd(DMA1_Stream3, DISABLE);	//ʧ��DMA

    //RX
    DMA_DeInit(DMA1_Stream1);
    while( DMA_GetCmdStatus(DMA1_Stream1) == ENABLE );			//�ȴ�DMA������

    dma.DMA_Channel            =     DMA_Channel_4;				//��DMA_Channel_5
    dma.DMA_PeripheralBaseAddr = 	(uint32_t)&(USART3->DR);
    dma.DMA_Memory0BaseAddr    = 	(uint32_t)UA3RxDMAbuf;
    dma.DMA_DIR                = 	DMA_DIR_PeripheralToMemory;	//���赽�ڴ�
    dma.DMA_BufferSize         = 	UA3RxDMAbuf_LEN;            //�������鳤�ȣ��Զ���
    dma.DMA_Mode               = 	DMA_Mode_Circular;			//ѭ������
    dma.DMA_Priority           = 	DMA_Priority_VeryHigh;
    DMA_Init(DMA1_Stream1, &dma);
    DMA_Cmd(DMA1_Stream1, ENABLE);	//ʹ��DMA
}
/*--------------------------------------------------------------------------
�������ܣ�����ͨ��
--------------------------------------------------------------------------*/
extern u8 UA4RxDMAbuf[];
void UART4_Configuration(void)
{
    GPIO_InitTypeDef	gpio= {0};
    USART_InitTypeDef	usart= {0};
    NVIC_InitTypeDef 	nvic= {0};
    DMA_InitTypeDef		dma= {0};

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);						//ʹ��UART4ʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_DMA1,ENABLE);	//ʹ��PC�˿�ʱ��

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);//Tx
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4);//Rx

    gpio.GPIO_OType = GPIO_OType_PP;//PP
    gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOA, &gpio);

    nvic.NVIC_IRQChannel 					= UART4_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority	= 0;		//��ռ���ȼ�
    nvic.NVIC_IRQChannelSubPriority			= 6;		//�����ȼ�
    nvic.NVIC_IRQChannelCmd					= ENABLE;	//IRQͨ��ʹ��
    NVIC_Init(&nvic);//����ָ���Ĳ�����ʼ��NVIC�Ĵ���

    usart.USART_BaudRate			= 460800;			  	//������
    usart.USART_WordLength			= USART_WordLength_8b;	//�ֳ�Ϊ8λ���ݸ�ʽ
    usart.USART_StopBits			= USART_StopBits_1;		//һ��ֹͣλ
    usart.USART_Parity				= USART_Parity_No;		//����żУ��λ
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//��Ӳ������������
    usart.USART_Mode				= USART_Mode_Tx|USART_Mode_Rx;		//�շ�ģʽ
    USART_Init(UART4, &usart);  //��ʼ������

    USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);	//���������ж�
    USART_Cmd(UART4, ENABLE);	//ʹ�ܴ���
    USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);
    USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE);

    //UART4_Tx
    DMA_DeInit(DMA1_Stream4);
    while( DMA_GetCmdStatus(DMA1_Stream4) == ENABLE );			//�ȴ�DMA������

    dma.DMA_Channel				=	DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr	=	(uint32_t)&(UART4->DR);
    dma.DMA_Memory0BaseAddr		=	NULL;//����
    dma.DMA_DIR					=	DMA_DIR_MemoryToPeripheral;	//�ڴ浽����
    dma.DMA_BufferSize			=	NULL;//����
    dma.DMA_PeripheralInc		=	DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc			=	DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize	=	DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize		=	DMA_MemoryDataSize_Byte;
    dma.DMA_Mode				=	DMA_Mode_Normal;			//��������
    dma.DMA_Priority			=	DMA_Priority_VeryHigh;
    dma.DMA_FIFOMode			=	DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold		=	DMA_FIFOThreshold_1QuarterFull;
    dma.DMA_MemoryBurst			=	DMA_MemoryBurst_Single;
    dma.DMA_PeripheralBurst		=	DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream4, &dma);
    DMA_Cmd(DMA1_Stream4, DISABLE);

    //UART4_Rx
    DMA_DeInit(DMA1_Stream2);
    while( DMA_GetCmdStatus(DMA1_Stream2) == ENABLE );			//�ȴ�DMA������

    dma.DMA_Channel            = 	DMA_Channel_4;				//��DMA_Channel_4
    dma.DMA_PeripheralBaseAddr = 	(uint32_t)&(UART4->DR);
    dma.DMA_Memory0BaseAddr    = 	(uint32_t)UA4RxDMAbuf;
    dma.DMA_DIR                = 	DMA_DIR_PeripheralToMemory;	//���赽�ڴ�
    dma.DMA_BufferSize         = 	UA4RxDMAbuf_LEN;
    dma.DMA_Mode               = 	DMA_Mode_Circular;			//ѭ������
    dma.DMA_Priority           = 	DMA_Priority_VeryHigh;
    DMA_Init(DMA1_Stream2, &dma);
    DMA_Cmd(DMA1_Stream2, ENABLE);
}
/*--------------------------------------------------------------------------
�������ܣ���
--------------------------------------------------------------------------*/
void USART6_Configuration(void)
{
    GPIO_InitTypeDef	gpio;
    USART_InitTypeDef	usart;
    NVIC_InitTypeDef 	nvic;
    DMA_InitTypeDef		dma;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_DMA2, ENABLE);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6); 	//TX�˿ڸ���
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6); 	//RX�˿ڸ���

    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd 	= GPIO_PuPd_UP;
    gpio.GPIO_Mode 	= GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_Pin 	= GPIO_Pin_6 | GPIO_Pin_7; 
    GPIO_Init(GPIOC, &gpio); 

    USART_DeInit(USART1);
    usart.USART_BaudRate			= 115200;       //����ʹ������Զ���
    usart.USART_WordLength		    = USART_WordLength_8b;
    usart.USART_StopBits			= USART_StopBits_1;
    usart.USART_Parity				= USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode			  	= USART_Mode_Rx | USART_Mode_Tx;    //���շ���
    USART_Init(USART6, &usart);

    USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);   //���������ж�
    USART_Cmd(USART6, ENABLE);                       //ʹ�ܴ���
    USART_DMACmd(USART6, USART_DMAReq_Tx, ENABLE);   //ʹ�ܴ��ڷ���DMA
    USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);   //ʹ�ܴ��ڽ���DMA

    nvic.NVIC_IRQChannel					= USART6_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority  = 0;
    nvic.NVIC_IRQChannelSubPriority			= 10;
    nvic.NVIC_IRQChannelCmd					= ENABLE;
    NVIC_Init(&nvic);

    //Tx
    DMA_DeInit(DMA2_Stream6);
    while( DMA_GetCmdStatus(DMA2_Stream6) == ENABLE );			//�ȴ�DMA������

    dma.DMA_Channel			      	=   DMA_Channel_5;
    dma.DMA_PeripheralBaseAddr	    =	(uint32_t)&(USART6->DR);
    dma.DMA_Memory0BaseAddr	  	    =	NULL;//����
    dma.DMA_DIR				        =	DMA_DIR_MemoryToPeripheral;	//�ڴ浽����
    dma.DMA_BufferSize	    		=	NULL;//����
    dma.DMA_PeripheralInc		    =	DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc		      	=	DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize	    =	DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize	    	=	DMA_MemoryDataSize_Byte;
    dma.DMA_Mode                    =	DMA_Mode_Normal;			//��������
    dma.DMA_Priority	      		=	DMA_Priority_VeryHigh;
    dma.DMA_FIFOMode		      	=	DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold	    	=	DMA_FIFOThreshold_1QuarterFull;
    dma.DMA_MemoryBurst		    	=	DMA_MemoryBurst_Single;
    dma.DMA_PeripheralBurst	    	=	DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream6, &dma);
    DMA_Cmd(DMA2_Stream6, DISABLE);	//ʧ��DMA

    //RX
    DMA_DeInit(DMA2_Stream1);
    while( DMA_GetCmdStatus(DMA2_Stream1) == ENABLE );			//�ȴ�DMA������

    dma.DMA_Channel            =   DMA_Channel_5;				//��DMA_Channel_5
    dma.DMA_PeripheralBaseAddr = 	(uint32_t)&(USART6->DR);
    dma.DMA_Memory0BaseAddr    = 	(uint32_t)UA6RxDMAbuf;
    dma.DMA_DIR                = 	DMA_DIR_PeripheralToMemory;	//���赽�ڴ�
    dma.DMA_BufferSize         = 	UA6RxDMAbuf_LEN;            //�������鳤�ȣ��Զ���
    dma.DMA_Mode               = 	DMA_Mode_Circular;			//ѭ������
    dma.DMA_Priority           = 	DMA_Priority_VeryHigh;
    DMA_Init(DMA2_Stream1, &dma);
    DMA_Cmd(DMA2_Stream1, ENABLE);	//ʹ��DMA
}
