#ifndef __OS_H__
#define __OS_H__

#include "hit_global_type.h"

//typedef enum {FALSE = 0, TRUE = !FALSE} bool;

typedef enum
{
    OS_READY,                           /*等待执行*/
    OS_FINISH,                          /*执行完毕*/
    OS_SUSPENDED,                       /*暂时挂起*/
} OS_TASK_STATE;

typedef struct
{
    const char*     Name;               //任务名称字符串
    void           (*Func)(void);       //任务函数指针
    OS_TASK_STATE   State;              //任务执行状态
    unsigned int    TimeDelay;          //任务结束延迟多长时间再次进入任务
    unsigned int    TimeExecuteLast;    //上一次执行任务的时刻
    unsigned int    TimeDiff;           //相邻两次任务的执行时间间隔
    unsigned int    TimeUsed;           //任务占用时间
    unsigned int    TimeUsedMost;       //任务占用时间
    unsigned int    TimeUsedLeast;      //任务占用时间
} ST_TAST;

typedef struct
{
    bool SysTickFlag;

    bool CAN1Flag;
    bool CAN2Flag;

    bool USART1Flag;
    bool USART2Flag;
    bool USART3Flag;
    bool UART4Flag;
    bool UART5Flag;
    bool USART6Flag;
} ST_IQRFLAG;


extern ST_IQRFLAG G_ST_IQRFlag;
extern ST_TAST G_ST_Task[];
extern u32 TotalTaskNum;
extern u32 TaskTotalTime;
extern u32 TaskTotalTimeMost;
extern u32 TaskTotalTimeLeast;

extern "C" void OS_RUN(void);

void IRQFlagPost(bool* IRQFlagParameter);
bool IRQFlagGet(bool* IRQFlagParameter);


#endif
