#include "temp_control.h"
ST_PID g_IMUTemp = {0,0,\
                    60,0.05,400,\
                    0,0,0,\
                    0,0,0,0,\
                    999,999,999,999,1
                   };

extern _sensor_st sensor;
extern s16 Real_Temp;
void Temp_Control(void)
{
//    if(Real_Temp < 30)
//    {
//        TIM3->CCR1 = (u16)999;
//    }
//    else
//    {
//        g_IMUTemp.fpDes = 35;
//        g_IMUTemp.fpFB = Real_Temp;
//        CalIWeakenPID(&g_IMUTemp);
//        if(g_IMUTemp.fpU >= 0) TIM3->CCR1 = (u16)g_IMUTemp.fpU;
//        else TIM3->CCR1 = 0;
//    }
}

