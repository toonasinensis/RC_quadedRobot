#include "wwdg.h"


/*--------------------------------------------------------------------------
�������ܣ����ڿ��Ź���ʼ��
��λ������������ֵ�����ϴ���ֵ �� 0x40����Ϊ0x3Fʱ��λ��
����ʵ�֣��´���ֵʱ�̶��ģ�Ϊ0x3F�����ϴ���ֵ�������ã�
          �� ��ǰֵΪ0x40ʱ�����жϷ�����
		  0x3F=�´���<����ֵ<=�ϴ���<=0x7F������ᱻ��λ
--------------------------------------------------------------------------*/
u8 WWDG_Counter = 0x6F;//��6λ��0x2F=10 1111(2)=47(10)
void WWDG_Configuration(void)
{
    NVIC_InitTypeDef nvic;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);

    //��ʱʱ��Ϊ4096*2^WWDG_Prescaler*(T[5:0]+1)/42M=4096*2^2*(0x2F + 1)/42M=18.72ms
    WWDG_SetPrescaler(WWDG_Prescaler_2);    //����Ԥ��Ƶϵ����
    WWDG_SetWindowValue(0x7F);              //�����ϴ���ֵ��<=0x7F,��
    WWDG_Enable(WWDG_Counter);              //ʹ�ܴ��ڿ��Ź�,ֵ��0x40-0x7F֮��

    nvic.NVIC_IRQChannel = WWDG_IRQn;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&nvic);

    WWDG_ClearFlag();
    WWDG_EnableIT();//ʹ���ж�
}


//�������ﵽ0x40,�Զ������ж�
void WWDG_IRQHandler()
{
    WWDG_SetCounter(WWDG_Counter);  //ι����ֵ��0x40-0x7F֮��
    WWDG_ClearFlag();
}



