#include "main.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_dma.h"

#define  ADC1_DR_Address    ((UINT32)0x4001244C)//这个地址可以去查数据手册，基地址为0x40012400+偏移量为0x4c

volatile USHORT16 AD_Value[6] = {0};
/****************************************************************************************************
函数名称: ADC_DMA_Configuration()
函数功能: 初始化ADC和DMA
输入参数: 无
返回参数: 无
备   注:  PA0——ADC123_IN0  Uc
          PA1——ADC123_IN1  Ub
          PA2——ADC123_IN2  Ua
          PB0——ADC12_IN8   Ic
          PB1——ADC12_IN9   Ia
          PC2——ADC123_IN12 Udc
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

    //初始化管脚
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    //NVIC初始化
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //初始化ADC相关参数
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//独立模式
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;//预分频4分频。ADCCLK=PCLK2/4=84/4=21Mhz,ADC时钟最好不要超过36Mhz
    ADC_CommonInit(&ADC_CommonInitStructure);//初始化

    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12位模式
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;//使能扫描模式对应多通道模式
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//关闭连续转换
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;

    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//右对齐
    ADC_InitStructure.ADC_NbrOfConversion = 6;
    ADC_Init(ADC1, &ADC_InitStructure);//ADC初始化

    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_3Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_3Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 3, ADC_SampleTime_3Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 4, ADC_SampleTime_3Cycles); //a
    ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 5, ADC_SampleTime_3Cycles); //c
    ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 6, ADC_SampleTime_3Cycles);

    // 开启ADC的DMA支持（要实现DMA功能，还需独立配置DMA通道等参数
    ADC_DMACmd(ADC1, ENABLE);

    // *****************DMA的设置****************** //
    DMA_DeInit(DMA2_Stream0);
    DMA_InitStructure.DMA_Channel             = DMA_Channel_0;
    DMA_InitStructure.DMA_PeripheralBaseAddr  = (u32)&ADC1->DR;//这里指明了ADC1的位置
    DMA_InitStructure.DMA_Memory0BaseAddr     = (u32)AD_Value;//DMA传值地址
    DMA_InitStructure.DMA_DIR                 = DMA_DIR_PeripheralToMemory;//决定外设作为目的地还是来源，此处当然是来源
    DMA_InitStructure.DMA_BufferSize          = 6;          //这里DMA只针对16个
    DMA_InitStructure.DMA_PeripheralInc       = DMA_PeripheralInc_Disable;//外设地址寄存器地址是否递增，此处选择不递增
    DMA_InitStructure.DMA_MemoryInc           = DMA_MemoryInc_Enable;         //内存地址是否递增，此处选择递增模式
    DMA_InitStructure.DMA_PeripheralDataSize  = DMA_PeripheralDataSize_HalfWord;//外设数据宽度，此处选择十六位宽度
    DMA_InitStructure.DMA_MemoryDataSize      = DMA_MemoryDataSize_HalfWord;//内存数据宽度，此处也是16位
    DMA_InitStructure.DMA_Mode                = DMA_Mode_Circular; //循环模式开启，Buffer写满后，自动回到初始地址开始传输
    DMA_InitStructure.DMA_Priority            = DMA_Priority_High;//DMA通道1拥有高优先级
    DMA_InitStructure.DMA_FIFOMode            = DMA_FIFOMode_Disable;//这里并不是内存到内存，而是外设ADC到内存
    DMA_InitStructure.DMA_FIFOThreshold       = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst         = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst     = DMA_PeripheralBurst_INC4;

    DMA_Init(DMA2_Stream0, &DMA_InitStructure);

    DMA_ITConfig(DMA2_Stream0,DMA_IT_TC,ENABLE);

    DMA_Cmd(DMA2_Stream0, ENABLE);//配置完成后，启动DMA通道

    ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);//没有这个DMA不好使

    ADC_Cmd(ADC1, ENABLE);// Enable ADC1

    ADC_SoftwareStartConv(ADC1); //使能指定的 ADC1 的软件转换启动功能


}

//volatile USHORT16 CurADValue[6] = {0};
//volatile USHORT16 VotADValue[6] = {0};
///****************************************************************************************************
//函数名称: ADC_DMA_Configuration()
//函数功能: 初始化ADC和DMA
//输入参数: 无
//返回参数: 无
//备   注:  PA0——ADC123_IN0  Uc
//          PA1——ADC123_IN1  Ub
//          PA2——ADC123_IN2  Ua
//          PB0——ADC12_IN8   Ib
//          PB1——ADC12_IN9   Ia
//          PC2——ADC123_IN12 Udc
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
//	//初始化管脚
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

//	//NVIC初始化
//  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);

//	//初始化ADC相关参数
//  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//独立模式
//	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
//	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;//预分频4分频。ADCCLK=PCLK2/4=84/4=21Mhz,ADC时钟最好不要超过36Mhz
//	ADC_CommonInit(&ADC_CommonInitStructure);//初始化
//
//  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12位模式
//	ADC_InitStructure.ADC_ScanConvMode = ENABLE;//使能扫描模式对应多通道模式
//	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//关闭连续转换
//	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
//
//	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//右对齐
//	ADC_InitStructure.ADC_NbrOfConversion = 6;
//	ADC_Init(ADC1, &ADC_InitStructure);//ADC初始化
//
//	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_3Cycles);
//	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_3Cycles);
//	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 3, ADC_SampleTime_3Cycles);
//	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 4, ADC_SampleTime_3Cycles);
//	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 5, ADC_SampleTime_3Cycles);
//  ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 6, ADC_SampleTime_3Cycles);
//
//	// 开启ADC的DMA支持（要实现DMA功能，还需独立配置DMA通道等参数
//  ADC_DMACmd(ADC1, ENABLE);

//	// *****************DMA的设置****************** //
//	DMA_DeInit(DMA2_Stream0);
//	DMA_InitStructure.DMA_Channel             = DMA_Channel_0;
//	DMA_InitStructure.DMA_PeripheralBaseAddr  = (u32)&ADC1->DR;//这里指明了ADC1的位置
//	DMA_InitStructure.DMA_Memory0BaseAddr     = (u32)CurADValue;//DMA传值地址
//	DMA_InitStructure.DMA_DIR                 = DMA_DIR_PeripheralToMemory;//决定外设作为目的地还是来源，此处当然是来源
//	DMA_InitStructure.DMA_BufferSize          = 6;          //这里DMA只针对16个
//	DMA_InitStructure.DMA_PeripheralInc       = DMA_PeripheralInc_Disable;//外设地址寄存器地址是否递增，此处选择不递增
//	DMA_InitStructure.DMA_MemoryInc           = DMA_MemoryInc_Enable;         //内存地址是否递增，此处选择递增模式
//	DMA_InitStructure.DMA_PeripheralDataSize  = DMA_PeripheralDataSize_HalfWord;//外设数据宽度，此处选择十六位宽度
//	DMA_InitStructure.DMA_MemoryDataSize      = DMA_MemoryDataSize_HalfWord;//内存数据宽度，此处也是16位
//	DMA_InitStructure.DMA_Mode                = DMA_Mode_Circular; //循环模式开启，Buffer写满后，自动回到初始地址开始传输
//	DMA_InitStructure.DMA_Priority            = DMA_Priority_High;//DMA通道1拥有高优先级
//	DMA_InitStructure.DMA_FIFOMode            = DMA_FIFOMode_Disable;//这里并不是内存到内存，而是外设ADC到内存
//	DMA_InitStructure.DMA_FIFOThreshold       = DMA_FIFOThreshold_1QuarterFull;
//	DMA_InitStructure.DMA_MemoryBurst         = DMA_MemoryBurst_Single;
//	DMA_InitStructure.DMA_PeripheralBurst     = DMA_PeripheralBurst_INC4;
//
//	DMA_Init(DMA2_Stream0, &DMA_InitStructure);
//
//	DMA_ITConfig(DMA2_Stream0,DMA_IT_TC,ENABLE);
//
//	DMA_Cmd(DMA2_Stream0, ENABLE);//配置完成后，启动DMA通道
//
//	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);//没有这个DMA不好使
//
//	ADC_Cmd(ADC1, ENABLE);// Enable ADC1

//	ADC_SoftwareStartConv(ADC1); //使能指定的 ADC1 的软件转换启动功能
//
//
//}

