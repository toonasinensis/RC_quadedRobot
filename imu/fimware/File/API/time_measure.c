#include "time_measure.h"

extern u16 tim5_cnt;

void Start_Catch(ST_TM* pTimer)
{
    pTimer->timer_freq = 84000000/TIM5_ARR/TIM5_PRE;
    pTimer->start_tick = tim5_cnt;
}

void End_Catch(ST_TM* pTimer)
{
    s32 delta_tick;
    pTimer->end_tick = tim5_cnt;
    delta_tick = pTimer->end_tick - pTimer->start_tick;
    if(delta_tick < -30000)
    {
        delta_tick += 65536;
    }
    pTimer->running_time = (float)delta_tick/(float)pTimer->timer_freq;

}



