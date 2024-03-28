#ifndef __SYSTEM_MONITOR_H__
#define __SYSTEM_MONITOR_H__

#include "hit_global_type.h"

typedef enum
{
    UART1_RX,//0
    UART2_RX,//1
    UART3_RX,//2
    UART4_RX,//3
    UART5_RX,//4
    UART6_RX,//5
    UDP_ENTER,//6
    UDP_REAL,//7
	  UDP_SEND,//8
	  IMU_RECV
} peripheral_list_e;

typedef enum
{
	SYS_INIT,
	SYS_RUN,
	SYS_ERROR
}system_state_e;

typedef struct
{
	u16             fps;
	u16             cnt;
}stMonitor;

typedef struct
{
	stMonitor RU_run_motor;
	stMonitor RD_run_motor;
	stMonitor LU_run_motor;
	stMonitor LD_run_motor;
	stMonitor FollowerWheel;
	stMonitor gyro_yaw;
	stMonitor remote_RX;
	stMonitor remote_TX;
	stMonitor vision_RX;
	stMonitor vision_TX;
}DataMonitor;

typedef struct
{
    u16 System_fps;
	
    u16 USART1_fps;
    u16 USART2_fps;
    u16 USART3_fps;
	  u16 UART4_fps;
	  u16 UART5_fps;
    u16 USART6_fps;
		u16 UART7_fps;
    u16 CAN1_fps;
    u16 CAN2_fps;
	
	  u16 motor_crl_Task_fps;
	  u16 navigation_task_fps;
    u16 action_task_fps;
		u16 remote_ctrl_task_fps;	  

    u16 System_cnt;
	
    u16 USART1_cnt;
    u16 USART2_cnt;
    u16 USART3_cnt;
		u16 UART4_cnt;
    u16 UART5_cnt;
    u16 USART6_cnt;
		u16 UART7_cnt;

    u16 CAN1_cnt;
    u16 CAN2_cnt;

    u16 motor_crl_Task_cnt;
	  u16 navigation_task_cnt;
    u16 action_task_cnt;
		u16 remote_ctrl_task_cnt;			

    u32 TaskTotalTime;
    u32 TaskTotalTimeMost;
    u32 TaskTotalTimeLeast;

    u16 CommuErrorBit;  //通信错误标志位
    u16 TaskWarningBit; //任务警告标志位
} SYSTEM_MONITOR;

extern SYSTEM_MONITOR system_monitor;
extern DataMonitor data_monitor;
extern system_state_e system_state;
void cal_fps_sys(SYSTEM_MONITOR  *sys);


#endif /* __SYSTEM_MONITOR_H__ */
