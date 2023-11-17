#include "system_monitor.h"
//#include "robot.h"

SYSTEM_MONITOR system_monitor = {0};    //ÏµÍ³¼àÊÓÆ÷

void cal_fps_sys(SYSTEM_MONITOR  *sys)
{
	
	sys->USART1_fps  = sys->USART1_cnt;
	sys->USART2_fps  = sys->USART2_cnt;
	sys->USART3_fps  = sys->USART3_cnt;
	sys->UART4_fps   = sys->UART4_cnt;
	sys->UART5_fps   = sys->UART5_cnt;
	sys->USART6_fps  = sys->USART6_cnt;

	
	
	
	
	sys->USART1_cnt = 0;
	sys->USART2_cnt = 0;

	sys->USART3_cnt = 0;
	sys->UART4_cnt=0;
	sys->UART5_cnt=0;
	sys->USART6_cnt=0;
}
//#include "udp_client.h"


