#ifndef __TASK_OS_H__
#define __TASK_OS_H__

#include "system_monitor.h"



void SystemMonitorTask(void);
void motor_ctrl_Task(void);
void udp_send_task(void);
void action_task(void);
void remote_ctrl_task(void);

//void gait_task(void);
//void DebugTask(void);

#endif

