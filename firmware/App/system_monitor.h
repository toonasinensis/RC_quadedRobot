#ifndef __SYSTEM_MONITOR_H__
#define __SYSTEM_MONITOR_H__

// #include "main.h"
#include "sys.h"
// #include "global_declare.h"

typedef enum {
  UART1_RX,  // 0
  UART2_RX,  // 1
  UART3_RX,  // 2
  UART4_RX,  // 3
  UART5_RX,  // 4
  UART6_RX,  // 5
  UDP_ENTER, // 6
  UDP_REAL,  // 7
  UDP_SEND,  // 8
  IMU_RECV
} peripheral_list_e;

typedef enum {
  SYS_IDLE,
  SYS_NORMAL,
  SYS_ERROR,
} system_state_e;

typedef struct {
  u16 System_fps;
  u16 USART1_fps;
  u16 USART2_fps;
  u16 USART3_fps;
  u16 UART4_fps;
  u16 UART5_fps;
  u16 UART5_data_fps;
  u16 USART6_fps;
  u16 UART7_fps;

  u16 CAN1_fps;
  u16 CAN1_Vision_fps;
  u16 CAN2_fps;

  u16 System_cnt;
  u16 USART1_cnt;
  u16 USART2_cnt;
  u16 USART3_cnt;
  u16 UART4_cnt;
  u16 UART5_cnt;
  u16 UART5_data_cnt;
  u16 USART6_cnt;
  u16 UART7_cnt;

  u16 CAN1_cnt;
  u16 CAN1_Vision_cnt;
  u16 CAN2_cnt;

  u32 TaskTotalTime;
  u32 TaskTotalTimeMost;
  u32 TaskTotalTimeLeast;

  u16 CommuErrorBit;  // 通信错误标志位
  u16 TaskWarningBit; // 任务警告标志位
} SYSTEM_MONITOR;

extern SYSTEM_MONITOR system_monitor;
void cal_fps_sys(SYSTEM_MONITOR *sys);

#endif /* __SYSTEM_MONITOR_H__ */
