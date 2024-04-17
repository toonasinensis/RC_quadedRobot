#ifndef __SYSTEMMONITOR_TASK_H__
#define __SYSTEMMONITOR_TASK_H__


#include "global_declare.h"



#define Calc_System_Monitor_fps(xx) do{  \
    system_monitor.xx##_fps = system_monitor.xx##_cnt; \
	system_monitor.xx##_cnt = 0; \
} while(0)

#define Calc_All_fps() do{  \
    Calc_System_Monitor_fps(System);\
  	Calc_System_Monitor_fps(USART1);\
	  Calc_System_Monitor_fps(USART2);\
    Calc_System_Monitor_fps(USART3);\
    Calc_System_Monitor_fps(UART4);\
    Calc_System_Monitor_fps(USART6);\
    Calc_System_Monitor_fps(IMUUpdateTask);\
    Calc_System_Monitor_fps(IMUSampleTask);\
	  Calc_System_Monitor_fps(TempControlTask);\
    Calc_System_Monitor_fps(GimbalTask);\
    Calc_System_Monitor_fps(SendDataTask);\
    Calc_System_Monitor_fps(LedTask);\
    Calc_System_Monitor_fps(DebugTask);\
    Calc_System_Monitor_fps(NotDefined1);\
    Calc_System_Monitor_fps(NotDefined2);\
    Calc_System_Monitor_fps(NotDefined3);\
    Calc_System_Monitor_fps(NotDefined4);\
} while(0)

void SystemErrorDetect(void);


#endif
