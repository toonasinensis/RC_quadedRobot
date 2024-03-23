#include "os.h"

/*--------------------------------------------------------------------

                ================================
                     ##### How to use #####
                ================================

任务执行机制：  G_ST_Task[]中序号低的执行时的优先级最高，
                高优先级任务不可剥夺正在执行的低优先级任务
                ，必须等待低优先级任务执行完毕方可执行高优
                先级的任务，因此低优先级任务最好不要占用过
                多使用时间。
系统时钟：      通过修改宏OS_TIME()可以指定系统时间，注意不
                同时钟来源可能使得G_ST_Task[i]时间相关变量
                的单位不同
使用方法：      步骤一：在宏OS_TIME()中修改系统时钟来源
                步骤二：在G_ST_Task[]结构体中添加任务
注意事项：      延迟1000us的任务，帧率可能会比1000小一点，
                这是因为延迟判断条件严格不小于所设置的延迟
                引起的，若需要准确的帧率，解决方法：将延迟
                改为合适的值，再直接限制每秒不超多少次运行
                次数
--------------------------------------------------------------------*/


/*-------------------------------------------------------------------
备    注：系统各任务占用总时间
-------------------------------------------------------------------*/
u32 TaskTotalTime;
u32 TaskTotalTimeMost;
u32 TaskTotalTimeLeast;

/*-------------------------------------------------------------------
备    注：系统时钟来源
-------------------------------------------------------------------*/
#define OS_TIME() TIM5->CNT

/*-------------------------------------------------------------------
备    注：系统任务结构体，注意与时间相关的变量与OS_TIME()单位相同
-------------------------------------------------------------------*/
//typedef enum {OS_READY,OS_FINISH,OS_SUSPENDED} OS_TASK_STATE;
ST_TAST G_ST_Task[] =
{
    {.Name="SystemMonitorTask", .Func=SystemMonitorTask,    .State=OS_READY,    .TimeDelay=1000*1000,   .TimeUsedLeast=0xFFFFFFFF},
//{.Name="IRQHandlerTask",    .Func=IRQHandlerTask,       .State=OS_READY,    .TimeDelay=1000*1,      .TimeUsedLeast=0xFFFFFFFF},
    {.Name="IMUSampleTask",     .Func=IMUSampleTask,        .State=OS_READY,    .TimeDelay=250,         .TimeUsedLeast=0xFFFFFFFF},
    {.Name="IMUUpdateTask",     .Func=IMUUpdateTask,        .State=OS_READY,    .TimeDelay=1000*1,      .TimeUsedLeast=0xFFFFFFFF},
    {.Name="TempControlTask",   .Func=TempControlTask,      .State=OS_READY,    .TimeDelay=1000*10,     .TimeUsedLeast=0xFFFFFFFF},
    {.Name="SendDataTask",      .Func=SendDataTask,         .State=OS_READY,    .TimeDelay=1000*1,      .TimeUsedLeast=0xFFFFFFFF},
    {.Name="GimbalTask",        .Func=GimbalTask,           .State=OS_READY,    .TimeDelay=1000*1,      .TimeUsedLeast=0xFFFFFFFF},
    {.Name="LedTask",           .Func=LedTask,              .State=OS_READY,    .TimeDelay=1000*100,    .TimeUsedLeast=0xFFFFFFFF},
//{.Name="DebugTask",         .Func=DebugTask,            .State=OS_READY,    .TimeDelay=1000*10,     .TimeUsedLeast=0xFFFFFFFF},
};
u32 TotalTaskNum = sizeof(G_ST_Task)/sizeof(ST_TAST);


/*-------------------------------------------------------------------
备    注：信号量
-------------------------------------------------------------------*/
ST_IQRFLAG G_ST_IQRFlag = {FALSE};







/*-------------------------------------------------------------------
函数功能：使用该函数将开启系统，并执行各个任务，且不可退出
备    注：使用inline关键字避免栈内存重复开辟造成的开销，
          关键字inline必须与函数定义体放在一起才能使函数成为内联，
          仅将inline 放在函数声明前面不起任何作用
-------------------------------------------------------------------*/
inline void OS_RUN(void)
{
    u32 i;
    u32 j;

    while(1)
    {
        for(i=0; i<TotalTaskNum; i++)
        {
            if(G_ST_Task[i].State == OS_READY)
            {
                G_ST_Task[i].TimeDiff = OS_TIME() - G_ST_Task[i].TimeExecuteLast;   //计算相邻两次任务的执行时间间隔
                G_ST_Task[i].TimeExecuteLast = OS_TIME();                           //记录上一次执行任务的时刻
                (*(void(*)())G_ST_Task[i].Func)();                                  //执行任务
                G_ST_Task[i].State = OS_FINISH;                                     //任务完成标志
                G_ST_Task[i].TimeUsed = OS_TIME() - G_ST_Task[i].TimeExecuteLast;   //计算任务消耗时间
                G_ST_Task[i].TimeUsedMost = G_ST_Task[i].TimeUsedMost > G_ST_Task[i].TimeUsed ? G_ST_Task[i].TimeUsedMost : G_ST_Task[i].TimeUsed;      //计算任务最多占用多长时间
                G_ST_Task[i].TimeUsedLeast = G_ST_Task[i].TimeUsedLeast < G_ST_Task[i].TimeUsed ? G_ST_Task[i].TimeUsedLeast : G_ST_Task[i].TimeUsed;    //计算任务最少占用多长时间
                TaskTotalTime += G_ST_Task[i].TimeUsed;
                TaskTotalTimeMost += G_ST_Task[i].TimeUsedMost;
                TaskTotalTimeLeast += G_ST_Task[i].TimeUsedLeast;
            }

            //每执行完一个任务重新计算最高优先级任务
            for(j=0; j<sizeof(G_ST_Task)/sizeof(ST_TAST); j++)
            {
                if( OS_TIME() - G_ST_Task[j].TimeExecuteLast >= G_ST_Task[j].TimeDelay &&
                        G_ST_Task[j].State != OS_SUSPENDED
                  )
                {
                    G_ST_Task[j].State = OS_READY;
                    if(i>j)
                    {
                        i=j-1;
                        break;
                    }
                }
            }
        }
    }
}

/*-------------------------------------------------------------------
函数功能：传递中断信号
备    注：一旦发出IRQFlag的值必然变为TRUE
-------------------------------------------------------------------*/
inline void IRQFlagPost(bool* IRQFlagParameter)
{
    *IRQFlagParameter = TRUE;
}

/*-------------------------------------------------------------------
函数功能：获取中断信号
备    注：一旦获取IRQFlag的值必然变为FALSE
-------------------------------------------------------------------*/
inline bool IRQFlagGet(bool* IRQFlagParameter)
{
    if(*IRQFlagParameter == TRUE)
    {
        *IRQFlagParameter = FALSE;  //数据用完就清零
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}
