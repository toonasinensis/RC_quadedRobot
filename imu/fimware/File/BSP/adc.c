#include "main.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_dma.h"

#define  ADC1_DR_Address    ((UINT32)0x4001244C)//�����ַ����ȥ�������ֲᣬ����ַΪ0x40012400+ƫ����Ϊ0x4c

volatile USHORT16 AD_Value[6] = {0};
/****************************************************************************************************
��������: ADC_DMA_Configuration()
��������: ��ʼ��ADC��DMA
�������: ��
���ز���: ��
��   ע:  PA0����ADC123_IN0  Uc
          PA1����ADC123_IN1  Ub
          PA2����ADC123_IN2  Ua
          PB0����ADC12_IN8   Ic
          PB1����ADC12_IN9   Ia
          PC2����ADC123_IN12 Udc
****************************************************************************************************/
void ADC_DMA_Configuration(void)
{
    ADC_InitTypeDef   ADC_InitStructure;
    GPIO_InitTypeDef  GPIO_InitStructure;
    DMA_InitTypeDef   DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    ADC_CommonInitTypeDef ADC_CommonInitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

    //��ʼ���ܽ�
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    //NVIC��ʼ��
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //��ʼ��ADC��ز���
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//����ģʽ
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;//Ԥ��Ƶ4��Ƶ��ADCCLK=PCLK2/4=84/4=21Mhz,ADCʱ����ò�Ҫ����36Mhz
    ADC_CommonInit(&ADC_CommonInitStructure);//��ʼ��

    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12λģʽ
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;//ʹ��ɨ��ģʽ��Ӧ��ͨ��ģʽ
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//�ر�����ת��
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;

    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//�Ҷ���
    ADC_InitStructure.ADC_NbrOfConversion = 6;
    ADC_Init(ADC1, &ADC_InitStructure);//ADC��ʼ��

    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_3Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_3Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 3, ADC_SampleTime_3Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 4, ADC_SampleTime_3Cycles); //a
    ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 5, ADC_SampleTime_3Cycles); //c
    ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 6, ADC_SampleTime_3Cycles);

    // ����ADC��DMA֧�֣�Ҫʵ��DMA���ܣ������������DMAͨ���Ȳ���
    ADC_DMACmd(ADC1, ENABLE);

    // *****************DMA������****************** //
    DMA_DeInit(DMA2_Stream0);
    DMA_InitStructure.DMA_Channel             = DMA_Channel_0;
    DMA_InitStructure.DMA_PeripheralBaseAddr  = (u32)&ADC1->DR;//����ָ����ADC1��λ��
    DMA_InitStructure.DMA_Memory0BaseAddr     = (u32)AD_Value;//DMA��ֵ��ַ
    DMA_InitStructure.DMA_DIR                 = DMA_DIR_PeripheralToMemory;//����������ΪĿ�ĵػ�����Դ���˴���Ȼ����Դ
    DMA_InitStructure.DMA_BufferSize          = 6;          //����DMAֻ���16��
    DMA_InitStructure.DMA_PeripheralInc       = DMA_PeripheralInc_Disable;//�����ַ�Ĵ�����ַ�Ƿ�������˴�ѡ�񲻵���
    DMA_InitStructure.DMA_MemoryInc           = DMA_MemoryInc_Enable;         //�ڴ��ַ�Ƿ�������˴�ѡ�����ģʽ
    DMA_InitStructure.DMA_PeripheralDataSize  = DMA_PeripheralDataSize_HalfWord;//�������ݿ�ȣ��˴�ѡ��ʮ��λ���
    DMA_InitStructure.DMA_MemoryDataSize      = DMA_MemoryDataSize_HalfWord;//�ڴ����ݿ�ȣ��˴�Ҳ��16λ
    DMA_InitStructure.DMA_Mode                = DMA_Mode_Circular; //ѭ��ģʽ������Bufferд�����Զ��ص���ʼ��ַ��ʼ����
    DMA_InitStructure.DMA_Priority            = DMA_Priority_High;//DMAͨ��1ӵ�и����ȼ�
    DMA_InitStructure.DMA_FIFOMode            = DMA_FIFOMode_Disable;//���ﲢ�����ڴ浽�ڴ棬��������ADC���ڴ�
    DMA_InitStructure.DMA_FIFOThreshold       = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst         = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst     = DMA_PeripheralBurst_INC4;

    DMA_Init(DMA2_Stream0, &DMA_InitStructure);

    DMA_ITConfig(DMA2_Stream0,DMA_IT_TC,ENABLE);

    DMA_Cmd(DMA2_Stream0, ENABLE);//������ɺ�����DMAͨ��

    ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);//û�����DMA����ʹ

    ADC_Cmd(ADC1, ENABLE);// Enable ADC1

    ADC_SoftwareStartConv(ADC1); //ʹ��ָ���� ADC1 �����ת����������


}

//volatile USHORT16 CurADValue[6] = {0};
//volatile USHORT16 VotADValue[6] = {0};
///****************************************************************************************************
//��������: ADC_DMA_Configuration()
//��������: ��ʼ��ADC��DMA
//�������: ��
//���ز���: ��
//��   ע:  PA0����ADC123_IN0  Uc
//          PA1����ADC123_IN1  Ub
//          PA2����ADC123_IN2  Ua
//          PB0����ADC12_IN8   Ib
//          PB1����ADC12_IN9   Ia
//          PC2����ADC123_IN12 Udc
//****************************************************************************************************/
//void ADC_DMA_Configuration(void)
//{
//	ADC_InitTypeDef   ADC_InitStructure;
//	GPIO_InitTypeDef  GPIO_InitStructure;
//	DMA_InitTypeDef   DMA_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//	ADC_CommonInitTypeDef ADC_CommonInitStructure;

//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
//
//	//��ʼ���ܽ�
//	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
//
//	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0 | GPIO_Pin_1;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
//
//	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_2;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
//	GPIO_Init(GPIOC, &GPIO_InitStructure);

//	//NVIC��ʼ��
//  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);

//	//��ʼ��ADC��ز���
//  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//����ģʽ
//	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
//	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;//Ԥ��Ƶ4��Ƶ��ADCCLK=PCLK2/4=84/4=21Mhz,ADCʱ����ò�Ҫ����36Mhz
//	ADC_CommonInit(&ADC_CommonInitStructure);//��ʼ��
//
//  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12λģʽ
//	ADC_InitStructure.ADC_ScanConvMode = ENABLE;//ʹ��ɨ��ģʽ��Ӧ��ͨ��ģʽ
//	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//�ر�����ת��
//	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
//
//	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//�Ҷ���
//	ADC_InitStructure.ADC_NbrOfConversion = 6;
//	ADC_Init(ADC1, &ADC_InitStructure);//ADC��ʼ��
//
//	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_3Cycles);
//	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_3Cycles);
//	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 3, ADC_SampleTime_3Cycles);
//	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 4, ADC_SampleTime_3Cycles);
//	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 5, ADC_SampleTime_3Cycles);
//  ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 6, ADC_SampleTime_3Cycles);
//
//	// ����ADC��DMA֧�֣�Ҫʵ��DMA���ܣ������������DMAͨ���Ȳ���
//  ADC_DMACmd(ADC1, ENABLE);

//	// *****************DMA������****************** //
//	DMA_DeInit(DMA2_Stream0);
//	DMA_InitStructure.DMA_Channel             = DMA_Channel_0;
//	DMA_InitStructure.DMA_PeripheralBaseAddr  = (u32)&ADC1->DR;//����ָ����ADC1��λ��
//	DMA_InitStructure.DMA_Memory0BaseAddr     = (u32)CurADValue;//DMA��ֵ��ַ
//	DMA_InitStructure.DMA_DIR                 = DMA_DIR_PeripheralToMemory;//����������ΪĿ�ĵػ�����Դ���˴���Ȼ����Դ
//	DMA_InitStructure.DMA_BufferSize          = 6;          //����DMAֻ���16��
//	DMA_InitStructure.DMA_PeripheralInc       = DMA_PeripheralInc_Disable;//�����ַ�Ĵ�����ַ�Ƿ�������˴�ѡ�񲻵���
//	DMA_InitStructure.DMA_MemoryInc           = DMA_MemoryInc_Enable;         //�ڴ��ַ�Ƿ�������˴�ѡ�����ģʽ
//	DMA_InitStructure.DMA_PeripheralDataSize  = DMA_PeripheralDataSize_HalfWord;//�������ݿ�ȣ��˴�ѡ��ʮ��λ���
//	DMA_InitStructure.DMA_MemoryDataSize      = DMA_MemoryDataSize_HalfWord;//�ڴ����ݿ�ȣ��˴�Ҳ��16λ
//	DMA_InitStructure.DMA_Mode                = DMA_Mode_Circular; //ѭ��ģʽ������Bufferд�����Զ��ص���ʼ��ַ��ʼ����
//	DMA_InitStructure.DMA_Priority            = DMA_Priority_High;//DMAͨ��1ӵ�и����ȼ�
//	DMA_InitStructure.DMA_FIFOMode            = DMA_FIFOMode_Disable;//���ﲢ�����ڴ浽�ڴ棬��������ADC���ڴ�
//	DMA_InitStructure.DMA_FIFOThreshold       = DMA_FIFOThreshold_1QuarterFull;
//	DMA_InitStructure.DMA_MemoryBurst         = DMA_MemoryBurst_Single;
//	DMA_InitStructure.DMA_PeripheralBurst     = DMA_PeripheralBurst_INC4;
//
//	DMA_Init(DMA2_Stream0, &DMA_InitStructure);
//
//	DMA_ITConfig(DMA2_Stream0,DMA_IT_TC,ENABLE);
//
//	DMA_Cmd(DMA2_Stream0, ENABLE);//������ɺ�����DMAͨ��
//
//	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);//û�����DMA����ʹ
//
//	ADC_Cmd(ADC1, ENABLE);// Enable ADC1

//	ADC_SoftwareStartConv(ADC1); //ʹ��ָ���� ADC1 �����ת����������
//
//
//}

