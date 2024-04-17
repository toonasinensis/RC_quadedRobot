#include "can.h"



/*-------------------------------------------------------
CAN1			|	Rx              |	Tx			    |
---------------------------------------------------------
����			|	PA11		 	|	PA2		    	|
�ж����ȼ�		|	��0��3		    |	��1��1		    |
������			|	42M			    |	42M			    |
---------------------------------------------------------
����			|   ���̵��������ϵͳ����������	    |
---------------------------------------------------------

---------------------------------------------------------
CAN2			|	Rx              |	Tx			    |
---------------------------------------------------------
����			|	PB12            |   PB13		        |
�ж����ȼ�		|	��0��4		    |	��1��2		    |
������			|	42M			    |	42M			    |
---------------------------------------------------------
����			|	���������pitch�����Yaw���	    |
-------------------------------------------------------*/


/*--------------------------------------------------------------------------
�������ܣ���ʼ��CAN1����Ϊ1M������
��    ע��PB8(CAN1_RX);PB9(CAN1_TX)
          BaudRate = 42MHz/Prescaler*(1+BS1+BS2)
--------------------------------------------------------------------------*/
void CAN1_Configuration(void)
{
    GPIO_InitTypeDef       gpio;
    NVIC_InitTypeDef       nvic;
    CAN_InitTypeDef        can1;
    CAN_FilterInitTypeDef  can1_fliter;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);   //ʹ��PB�˿�ʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,  ENABLE);   //ʹ��CAN1ʱ��,42MHz

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);

    gpio.GPIO_Pin   = GPIO_Pin_11 | GPIO_Pin_12;
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_Init(GPIOA, &gpio);

    nvic.NVIC_IRQChannel                    = CAN1_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority  = 0;        //��ռ���ȼ�
    nvic.NVIC_IRQChannelSubPriority         = 3;        //�����ȼ�
    nvic.NVIC_IRQChannelCmd                 = ENABLE;   //IRQͨ��ʹ��
    NVIC_Init(&nvic);   //����ָ���Ĳ�����ʼ��NVIC�Ĵ���

    /****************************************CAN��Ԫ��ʼ��*****************************************/
    CAN_DeInit(CAN1);
    CAN_StructInit(&can1);
    /*CAN�����ƼĴ�������CAN_MCR��*/
    can1.CAN_TTCM = DISABLE;        //ʱ�䴥��ͨ��ģʽ
    can1.CAN_ABOM = DISABLE;        //�Զ������߹رչ���
    can1.CAN_AWUM = DISABLE;        //�Զ�����ģʽ
    can1.CAN_NART = DISABLE;        //��ֹ�Զ��ط���
    can1.CAN_RFLM = DISABLE;        //����FIFO����ģʽ
    can1.CAN_TXFP = ENABLE;         //����FIFIO���ȼ�
    /*CANλʱ��Ĵ�����CAN_BTR��*/
    /*CAN1������=42MHz/3/(9+4+1)=1MHz*/
    can1.CAN_Mode = CAN_Mode_Normal;//��ͨģʽ
    can1.CAN_SJW  = CAN_SJW_1tq;    //����ͬ����Ծ���
    can1.CAN_BS1  = CAN_BS1_9tq;    //ʱ���1
    can1.CAN_BS2  = CAN_BS2_4tq;    //ʱ���2
    can1.CAN_Prescaler = 3;         //��Ƶϵ��
    CAN_Init(CAN1, &can1);          //����ָ���Ĳ�����ʼ��CAN�Ĵ���
    /****************************************CAN��������ʼ��****************************************/
    can1_fliter.CAN_FilterNumber         = 0;                       //������0
    can1_fliter.CAN_FilterMode           = CAN_FilterMode_IdMask;   //��ʶ��������Ϊ����ģʽ
    can1_fliter.CAN_FilterScale          = CAN_FilterScale_32bit;   //һ��32λ��ʶ��������
    can1_fliter.CAN_FilterIdHigh         = 0x0000;                  //32λ��ʶ���������ĸ�16λ
    can1_fliter.CAN_FilterIdLow          = 0x0000;                  //32λ��ʶ���������ĵ�16λ
    can1_fliter.CAN_FilterMaskIdHigh     = 0x0000;                  //�����������16λ
    can1_fliter.CAN_FilterMaskIdLow      = 0x0000;                  //�����������16λ
    can1_fliter.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;        //������0������FIFO0
    can1_fliter.CAN_FilterActivation     = ENABLE;                  //���������
    CAN_FilterInit(&can1_fliter);             //����ָ���Ĳ�����ʼ��CAN_Filter�Ĵ���

    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);  //�����ж�:FIFO 0��Ϣ�����ж�
}


/*--------------------------------------------------------------------------
�������ܣ���CAN2����Ϊ1M������
��    ע��PB5(CAN2_RX);PB6(CAN2_TX)
          BaudRate = 42MHz/Prescaler*(1+BS1+BS2)
--------------------------------------------------------------------------*/
void CAN2_Configuration(void)
{
    GPIO_InitTypeDef       gpio;
    NVIC_InitTypeDef       nvic;
    CAN_InitTypeDef        can2;
    CAN_FilterInitTypeDef  can2_filter;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);   //ʹ��PB�˿�ʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,  ENABLE);   //ʹ��CAN1ʱ��*ע��*������ʹ��can2,����ʹ��can1��ʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2,  ENABLE);   //ʹ��CAN2ʱ��,42MHz

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2);

    gpio.GPIO_Pin   = GPIO_Pin_12 | GPIO_Pin_13;
    gpio.GPIO_Mode  = GPIO_Mode_AF; //����ģʽ
    GPIO_Init(GPIOB, &gpio);        //�����趨������ʼ��GPIOB

    nvic.NVIC_IRQChannel                   = CAN2_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;         //��ռ���ȼ�   1101
    nvic.NVIC_IRQChannelSubPriority        = 4;         //�����ȼ�
    nvic.NVIC_IRQChannelCmd                = ENABLE;    //IRQͨ��ʹ��
    NVIC_Init(&nvic);   //����ָ���Ĳ�����ʼ��NVIC�Ĵ���

    /****************************************CAN��Ԫ��ʼ��*****************************************/
    CAN_DeInit(CAN2);
    CAN_StructInit(&can2);
    /*CAN�����ƼĴ�������CAN_MCR��*/
    can2.CAN_TTCM = DISABLE;//ʱ�䴥��ͨ��ģʽ
    can2.CAN_ABOM = DISABLE;//�Զ������߹رչ���
    can2.CAN_AWUM = DISABLE;//�Զ�����ģʽ
    can2.CAN_NART = DISABLE;//��ֹ�Զ��ط���
    can2.CAN_RFLM = DISABLE;//����FIFO����ģʽ
    can2.CAN_TXFP = ENABLE; //����FIFIO���ȼ�
    /*CANλʱ��Ĵ�����CAN_BTR��*/
    /*CAN1������=42MHz/3/(9+4+1)=1MHz*/
    can2.CAN_Mode = CAN_Mode_Normal;//��ͨģʽ
    can2.CAN_SJW  = CAN_SJW_1tq;    //����ͬ����Ծ���
    can2.CAN_BS1  = CAN_BS1_9tq;    //ʱ���1
    can2.CAN_BS2  = CAN_BS2_4tq;    //ʱ���2
    can2.CAN_Prescaler = 3;         //��Ƶϵ��
    CAN_Init(CAN2, &can2);          //����ָ���Ĳ�����ʼ��CAN�Ĵ���
    /*CAN�������Ĵ���*/
    can2_filter.CAN_FilterNumber         = 14;//������14
    can2_filter.CAN_FilterMode           = CAN_FilterMode_IdMask;   //��ʶ��������Ϊ����ģʽ
    can2_filter.CAN_FilterScale          = CAN_FilterScale_32bit;   //һ��32λ��ʶ��������
    can2_filter.CAN_FilterIdHigh         = 0x0000;                  //32λ��ʶ���������ĸ�16λ
    can2_filter.CAN_FilterIdLow          = 0x0000;                  //32λ��ʶ���������ĵ�16λ
    can2_filter.CAN_FilterMaskIdHigh     = 0x0000;                  //�����������16λ
    can2_filter.CAN_FilterMaskIdLow      = 0x0000;                  //�����������16λ
    can2_filter.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;        //������0������FIFO0
    can2_filter.CAN_FilterActivation     = ENABLE;                  //���������
    CAN_FilterInit(&can2_filter);//����ָ���Ĳ�����ʼ��CAN_Filter�Ĵ���

    /*CAN�ж�ʹ�ܼĴ�����CAN_IER��*/
    CAN_ITConfig(CAN2,CAN_IT_FMP0, ENABLE); //�����ж�:FIFO 0��Ϣ�����ж�
}
