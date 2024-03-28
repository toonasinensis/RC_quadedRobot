#ifndef __OS_H__
#define __OS_H__

#include "hit_global_type.h"

//typedef enum {FALSE = 0, TRUE = !FALSE} bool;

typedef enum
{
    OS_READY,                           /*�ȴ�ִ��*/
    OS_FINISH,                          /*ִ�����*/
    OS_SUSPENDED,                       /*��ʱ����*/
} OS_TASK_STATE;

typedef struct
{
    const char*     Name;               //���������ַ���
    void           (*Func)(void);       //������ָ��
    OS_TASK_STATE   State;              //����ִ��״̬
    unsigned int    TimeDelay;          //��������ӳٶ೤ʱ���ٴν�������
    unsigned int    TimeExecuteLast;    //��һ��ִ�������ʱ��
    unsigned int    TimeDiff;           //�������������ִ��ʱ����
    unsigned int    TimeUsed;           //����ռ��ʱ��
    unsigned int    TimeUsedMost;       //����ռ��ʱ��
    unsigned int    TimeUsedLeast;      //����ռ��ʱ��
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
