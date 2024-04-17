#include "wwdg.h"


/*--------------------------------------------------------------------------
函数功能：窗口看门狗初始化
复位条件：计数器值大于上窗口值 或 0x40跳变为0x3F时复位。
具体实现：下窗口值时固定的，为0x3F，而上窗口值可以设置，
          当 当前值为0x40时唤醒中断服务函数
		  0x3F=下窗口<计数值<=上窗口<=0x7F，否则会被复位
--------------------------------------------------------------------------*/
u8 WWDG_Counter = 0x6F;//低6位是0x2F=10 1111(2)=47(10)
void WWDG_Configuration(void)
{
    NVIC_InitTypeDef nvic;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);

    //定时时间为4096*2^WWDG_Prescaler*(T[5:0]+1)/42M=4096*2^2*(0x2F + 1)/42M=18.72ms
    WWDG_SetPrescaler(WWDG_Prescaler_2);    //设置预分频系数，
    WWDG_SetWindowValue(0x7F);              //设置上窗口值，<=0x7F,上
    WWDG_Enable(WWDG_Counter);              //使能窗口看门狗,值在0x40-0x7F之间

    nvic.NVIC_IRQChannel = WWDG_IRQn;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&nvic);

    WWDG_ClearFlag();
    WWDG_EnableIT();//使能中断
}


//计数器达到0x40,自动唤醒中断
void WWDG_IRQHandler()
{
    WWDG_SetCounter(WWDG_Counter);  //喂狗，值在0x40-0x7F之间
    WWDG_ClearFlag();
}



