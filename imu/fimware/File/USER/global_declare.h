#ifndef __GLOBAL_DECLARE_H__
#define __GLOBAL_DECLARE_H__

#include "main.h"
#include "stm32f4xx.h"
#include "gimbal_control_types.h"

/*--------------------ѡ���ͣ�����--------------------*/
//#define Infantry_A  //����B��
//#define Infantry_B  //����B��  ��ƽ��
//#define Infantry_C  //����C��
//#define Infantry_E  //����E��
#define robocon_r2
/*--------------------ѡ���ͣ�����--------------------*/

typedef enum {INIT = 0, NORMAL = 1, CALIBRATION = 2} IMU_MODE;

typedef struct
{
    u16 System_fps;
    u16 USART1_fps;
    u16 USART2_fps;
    u16 USART3_fps;
    u16 UART4_fps;
    u16 USART4_Xms_fps;
    u16 USART6_fps;

    u16 IMUSampleTask_fps;
    u16 IMUUpdateTask_fps;
    u16 TempControlTask_fps;
    u16 GimbalTask_fps;
    u16 SendDataTask_fps;
    u16 LedTask_fps;
    u16 DebugTask_fps;

    u16 NotDefined1_fps;
    u16 NotDefined2_fps;
    u16 NotDefined3_fps;
    u16 NotDefined4_fps;

    u16 System_cnt;
    u16 USART1_cnt;
    u16 USART2_cnt;
    u16 USART3_cnt;
    u16 UART4_cnt;
    u16 USART4_Xms_cnt;
    u16 USART6_cnt;

    u16 IMUSampleTask_cnt;
    u16 IMUUpdateTask_cnt;
    u16 TempControlTask_cnt;
    u16 GimbalTask_cnt;
    u16 SendDataTask_cnt;
    u16 LedTask_cnt;
    u16 DebugTask_cnt;

    u16 NotDefined1_cnt;
    u16 NotDefined2_cnt;
    u16 NotDefined3_cnt;
    u16 NotDefined4_cnt;

    u32 TaskTotalTime;
    u32 TaskTotalTimeMost;
    u32 TaskTotalTimeLeast;

    u16 CommuErrorBit;  //ͨ�Ŵ����־λ
    u16 TaskWarningBit; //���񾯸��־λ
} SYSTEM_MONITOR;

extern SYSTEM_MONITOR system_monitor;
extern u32 SteeringEnginePWM_High;
extern u32 SteeringEnginePWM_Low;


#endif
