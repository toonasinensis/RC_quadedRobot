#include "systemmonitor_task.h"


/*----------------------------------------------------------------------------------------
功能：异常情况监测工具
----------------------------------------------------------------------------------------*/
void SystemErrorDetect(void)
{
    Calc_All_fps();
    system_monitor.TaskTotalTime = TaskTotalTime;
    system_monitor.TaskTotalTimeMost = TaskTotalTimeMost;
    system_monitor.TaskTotalTimeLeast = TaskTotalTimeLeast;
    TaskTotalTime = 0;
    TaskTotalTimeMost = 0;
    TaskTotalTimeLeast = 0;
}




