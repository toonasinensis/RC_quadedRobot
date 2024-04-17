#ifndef __TIME_MEASURE__
#define __TIME_MEASURE__

#include "main.h"

typedef struct
{
    u16 start_tick;
    u16 end_tick;
    float running_time;
    u32 timer_freq;
} ST_TM;

void Start_Catch(ST_TM* pTimer);
void Start_Catch(ST_TM* pTimer);


#endif

