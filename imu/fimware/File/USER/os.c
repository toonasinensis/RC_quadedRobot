#include "os.h"

/*--------------------------------------------------------------------

                ================================
                     ##### How to use #####
                ================================

����ִ�л��ƣ�  G_ST_Task[]����ŵ͵�ִ��ʱ�����ȼ���ߣ�
                �����ȼ����񲻿ɰ�������ִ�еĵ����ȼ�����
                ������ȴ������ȼ�����ִ����Ϸ���ִ�и���
                �ȼ���������˵����ȼ�������ò�Ҫռ�ù�
                ��ʹ��ʱ�䡣
ϵͳʱ�ӣ�      ͨ���޸ĺ�OS_TIME()����ָ��ϵͳʱ�䣬ע�ⲻ
                ͬʱ����Դ����ʹ��G_ST_Task[i]ʱ����ر���
                �ĵ�λ��ͬ
ʹ�÷�����      ����һ���ں�OS_TIME()���޸�ϵͳʱ����Դ
                ���������G_ST_Task[]�ṹ�����������
ע�����      �ӳ�1000us������֡�ʿ��ܻ��1000Сһ�㣬
                ������Ϊ�ӳ��ж������ϸ�С�������õ��ӳ�
                ����ģ�����Ҫ׼ȷ��֡�ʣ�������������ӳ�
                ��Ϊ���ʵ�ֵ����ֱ������ÿ�벻�����ٴ�����
                ����
--------------------------------------------------------------------*/


/*-------------------------------------------------------------------
��    ע��ϵͳ������ռ����ʱ��
-------------------------------------------------------------------*/
u32 TaskTotalTime;
u32 TaskTotalTimeMost;
u32 TaskTotalTimeLeast;

/*-------------------------------------------------------------------
��    ע��ϵͳʱ����Դ
-------------------------------------------------------------------*/
#define OS_TIME() TIM5->CNT

/*-------------------------------------------------------------------
��    ע��ϵͳ����ṹ�壬ע����ʱ����صı�����OS_TIME()��λ��ͬ
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
��    ע���ź���
-------------------------------------------------------------------*/
ST_IQRFLAG G_ST_IQRFlag = {FALSE};







/*-------------------------------------------------------------------
�������ܣ�ʹ�øú���������ϵͳ����ִ�и��������Ҳ����˳�
��    ע��ʹ��inline�ؼ��ֱ���ջ�ڴ��ظ�������ɵĿ�����
          �ؼ���inline�����뺯�����������һ�����ʹ������Ϊ������
          ����inline ���ں�������ǰ�治���κ�����
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
                G_ST_Task[i].TimeDiff = OS_TIME() - G_ST_Task[i].TimeExecuteLast;   //�����������������ִ��ʱ����
                G_ST_Task[i].TimeExecuteLast = OS_TIME();                           //��¼��һ��ִ�������ʱ��
                (*(void(*)())G_ST_Task[i].Func)();                                  //ִ������
                G_ST_Task[i].State = OS_FINISH;                                     //������ɱ�־
                G_ST_Task[i].TimeUsed = OS_TIME() - G_ST_Task[i].TimeExecuteLast;   //������������ʱ��
                G_ST_Task[i].TimeUsedMost = G_ST_Task[i].TimeUsedMost > G_ST_Task[i].TimeUsed ? G_ST_Task[i].TimeUsedMost : G_ST_Task[i].TimeUsed;      //�����������ռ�ö೤ʱ��
                G_ST_Task[i].TimeUsedLeast = G_ST_Task[i].TimeUsedLeast < G_ST_Task[i].TimeUsed ? G_ST_Task[i].TimeUsedLeast : G_ST_Task[i].TimeUsed;    //������������ռ�ö೤ʱ��
                TaskTotalTime += G_ST_Task[i].TimeUsed;
                TaskTotalTimeMost += G_ST_Task[i].TimeUsedMost;
                TaskTotalTimeLeast += G_ST_Task[i].TimeUsedLeast;
            }

            //ÿִ����һ���������¼���������ȼ�����
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
�������ܣ������ж��ź�
��    ע��һ������IRQFlag��ֵ��Ȼ��ΪTRUE
-------------------------------------------------------------------*/
inline void IRQFlagPost(bool* IRQFlagParameter)
{
    *IRQFlagParameter = TRUE;
}

/*-------------------------------------------------------------------
�������ܣ���ȡ�ж��ź�
��    ע��һ����ȡIRQFlag��ֵ��Ȼ��ΪFALSE
-------------------------------------------------------------------*/
inline bool IRQFlagGet(bool* IRQFlagParameter)
{
    if(*IRQFlagParameter == TRUE)
    {
        *IRQFlagParameter = FALSE;  //�������������
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}
