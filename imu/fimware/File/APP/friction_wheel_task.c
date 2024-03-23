#include "friction_wheel_task.h"

ST_SMC smc1 = {
    .b = 4.49939,
    .eps = 5000,
    .dead = 5,
    .gain = 40,
    .fpDes = 0,
    .fpFB = 0,
    .fpU = 0,
    .fpUMax = 15000,

    .SmcTd.h = 0.001,
    .SmcTd.r = 5000,
    .SmcTd.T = 0.001,
    .SmcTd.aim = 0,
    .SmcTd.x = 0,
    .SmcTd.x1 = 0,
    .SmcTd.x2 = 0
};

ST_SMC smc2 = {
    .b = 4.49939,
    .eps = 5000,
    .dead = 5,
    .gain = 40,
    .fpDes = 0,
    .fpFB = 0,
    .fpU = 0,
    .fpUMax = 15000,

    .SmcTd.h = 0.001,
    .SmcTd.r = 5000,
    .SmcTd.T = 0.001,
    .SmcTd.aim = 0,
    .SmcTd.x = 0,
    .SmcTd.x1 = 0,
    .SmcTd.x2 = 0
};

float CM1_TEMP;
float CM2_TEMP;

u8 State_Flag = 0x00;
float SMC_J;
float SMC_f;
u32 Inc_time;
u32 Dec_time;
float Real_Temp1;
float Real_Temp2;
/*----------------------------------------------------------------------------------------
函数名：FrictionWheelControl()
----------------------------------------------------------------------------------------*/
void FrictionWheelControl(void)
{
    static u8 Auto_Phase = 0;
    static u32 time_tick = 0;

    float Inc_k = 0;
    float Dec_k = 0;

    static u8 Diff_Coe1 = 0;
    static u8 Diff_Coe2 = 0;
    static u8 Last_CM1_TEMP = 0;
    static u8 Last_CM2_TEMP = 0;
    static bool RISE_0_FALL1 = FALSE;
    static bool RISE_0_FALL2 = FALSE;

    static u32 tick1 = 0;
    static u32 tick2 = 0;
    static u32 Last_Time1 = 0;
    static u32 Last_Time2 = 0;

    /*------------------------------温度矫正------------------------------*/
    if(Last_CM1_TEMP < CM1_TEMP)
    {
        Diff_Coe1 = CM1_TEMP - Last_CM1_TEMP;
        Last_Time1 = tick1;
        tick1 = 0;
        Real_Temp1 = CM1_TEMP;
        RISE_0_FALL1 = TRUE;
    }
    else if(Last_CM1_TEMP > CM1_TEMP)
    {
        Diff_Coe1 = CM1_TEMP - Last_CM1_TEMP;
        Last_Time1 = tick1;
        tick1 = 0;
        Real_Temp1 = CM1_TEMP;
        RISE_0_FALL1 = FALSE;
    }
    else
    {
        tick1++;
        if(RISE_0_FALL1)
        {
            if(Real_Temp1 < CM1_TEMP+1)
                Real_Temp1 += (float)Diff_Coe1/(float)Last_Time1;
            else
                Real_Temp1  = CM1_TEMP+1.1f;
        }
        else
        {
            if(Real_Temp1 > CM1_TEMP-1)
                Real_Temp1 -= (float)Diff_Coe1/(float)Last_Time1;
            else
                Real_Temp1  = CM1_TEMP-1.1f;
        }
    }

    if(Last_CM2_TEMP < CM2_TEMP)
    {
        Diff_Coe2 = CM2_TEMP - Last_CM2_TEMP;
        Last_Time2 = tick2;
        tick2 = 0;
        Real_Temp2 = CM2_TEMP;
        RISE_0_FALL2 = TRUE;
    }
    else if(Last_CM2_TEMP > CM2_TEMP)
    {
        Diff_Coe2 = CM2_TEMP - Last_CM2_TEMP;
        Last_Time2 = tick2;
        tick2 = 0;
        Real_Temp2 = CM2_TEMP;
        RISE_0_FALL2 = FALSE;
    }
    else
    {
        tick2++;
        if(RISE_0_FALL2)
        {
            if(Real_Temp2 < CM2_TEMP+1)
                Real_Temp2 += (float)Diff_Coe2/(float)Last_Time2;
            else
                Real_Temp2  = CM2_TEMP+1.1f;
        }
        else
        {
            if(Real_Temp2 > CM2_TEMP-1)
                Real_Temp2 -= (float)Diff_Coe2/(float)Last_Time2;
            else
                Real_Temp2  = CM2_TEMP-1.1f;
        }
    }

    /*------------------------------速度控制------------------------------*/
    if(RC_ON)                                       //正常模式
    {
        ShootMode_Select();

        SlidingModeCtrler(&smc1);
        SlidingModeCtrler(&smc2);
    }
    else if(g_stTestFlag.FrictionTestFlag == TRUE)  //测试模式
    {
        if(Auto_Phase == 1)
        {
            if(smc1.fpFB > 6000)
            {
                Inc_time = time_tick;
                time_tick = 0;
                Auto_Phase = 2;
            }
            else
            {
                smc1.fpU = 500;
                time_tick++;
            }
        }
        else if(Auto_Phase == 2)
        {
            if(smc1.fpFB < 100)
            {
                Dec_time = time_tick;
                time_tick = 0;
                Auto_Phase = 0;
            }
            else
            {
                smc1.fpU = 0;
                time_tick++;
            }
        }
        else
        {
            Inc_k = 6000.0f*1000.0f/Inc_time;
            Dec_k = 5900.0f*1000.0f/Dec_time;

            SMC_J = 500/(Inc_k + Dec_k);
            SMC_f = SMC_J*Dec_k;
        }
    }
    else                                            //安全模式
    {
        time_tick = 0;
        Auto_Phase = 1;

        smc1.fpDes = 0;
        smc2.fpDes = 0;
        SlidingModeCtrler(&smc1);
        SlidingModeCtrler(&smc2);
    }

    Last_CM1_TEMP = CM1_TEMP;
    Last_CM2_TEMP = CM2_TEMP;
}

float SatFunc(float in, float d)
{
    if(fabs(in) >= d) return Sign_Judge(in);
    else return in/d;
}

/*-------------------------------------------------------------------------------------------------
函数功能：滑模控制
-------------------------------------------------------------------------------------------------*/
void SlidingModeCtrler(ST_SMC* pst_Smc)
{
    pst_Smc->SmcTd.aim = pst_Smc->fpDes;
    TD_Function(&pst_Smc->SmcTd);
    pst_Smc->fpE = pst_Smc->SmcTd.x1 - pst_Smc->fpFB;
    pst_Smc->fpU = 1/pst_Smc->b * (pst_Smc->SmcTd.x2 + pst_Smc->eps*SatFunc(pst_Smc->fpE,pst_Smc->dead) + pst_Smc->gain*pst_Smc->fpE);
    pst_Smc->fpU = Clip(pst_Smc->fpU, -pst_Smc->fpUMax, pst_Smc->fpUMax);
}

/*-------------------------------------------------------------------------------------------------
函数功能：射速模式选择
备注：非自爆模式根据底盘目标转速转，自爆强制低射速
-------------------------------------------------------------------------------------------------*/
void ShootMode_Select(void)
{
    if(G_ST_IMU.Receive.SelfBoom!=0xff)
    {
        switch(G_ST_IMU.Receive.FrictionSpeed_Des)
        {
        case FrictionSpeed_Zero:
        {
            smc1.fpDes = 0;
            smc2.fpDes = 0;
            break;
        }
        case FrictionSpeed_High:
        {
            smc1.fpDes = +HighSpeed1;
            smc2.fpDes = -HighSpeed2;
            break;
        }
        case FrictionSpeed_Low:
        {
            smc1.fpDes = +LowSpeed1;
            smc2.fpDes = -LowSpeed2;
            break;
        }
        case FrictionSpeed_Lowest:
        {
            smc1.fpDes = +LowestSpeed1;
            smc2.fpDes = -LowestSpeed2;
            break;
        }
        default:
        {
            smc1.fpDes = 0;
            smc2.fpDes = 0;
            break;
        }
        }
    }
    else
    {
        smc1.fpDes = +LowSpeed1;
        smc2.fpDes = -LowSpeed2;
    }
}
