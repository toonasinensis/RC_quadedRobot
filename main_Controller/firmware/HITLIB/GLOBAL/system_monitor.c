#include "system_monitor.h"

SYSTEM_MONITOR system_monitor = {0};    //ÏµÍ³¼àÊÓÆ÷
DataMonitor data_monitor = {0};
system_state_e system_state = SYS_INIT;

void cal_fps_sys(SYSTEM_MONITOR  *sys)
{
	sys->System_fps  = sys->System_cnt;
	sys->System_cnt = 0;

	sys->USART1_fps  = sys->USART1_cnt;
	sys->USART2_fps  = sys->USART2_cnt;
	sys->USART3_fps  = sys->USART3_cnt;
	sys->UART4_fps   = sys->UART4_cnt;
	sys->UART5_fps   = sys->UART5_cnt;
	sys->USART6_fps  = sys->USART6_cnt;
	sys->motor_crl_Task_fps    = sys->motor_crl_Task_cnt;
	sys->navigation_task_fps   = sys->navigation_task_cnt;
	sys->action_task_fps       = sys->action_task_cnt;
	sys->remote_ctrl_task_fps  = sys->remote_ctrl_task_cnt;

	sys->USART1_cnt = 0;
	sys->USART2_cnt = 0;
	sys->USART3_cnt = 0;
	sys->UART4_cnt=0;
	sys->UART5_cnt=0;
	sys->USART6_cnt=0;
	sys->motor_crl_Task_cnt = 0;
	sys->navigation_task_cnt = 0;
	sys->action_task_cnt = 0;
	sys->remote_ctrl_task_cnt = 0;
	
	
}


