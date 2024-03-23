/**********************************************************************************************************
版权声明： RoboMasters(HITCRT_iHiter 战队)
文件名： rm_algorithm.c
最近修改日期： 2018/03/26
版本： 3.1
-----------------------------------------------------------------------------------------------------------
模块描述：该模块定义了RM常用的算法。
-----------------------------------------------------------------------------------------------------------
修改记录：
作者                              时间         版本   说明
彭季超、彭毓兴、张冰、李四林    2016/01/25     1.0     建立此文件
FengYun                         2017/03/08	   2.0	   更新了PID算法以适应新的PID结构体，将原先增量式算法改
                                                       为位置式，完善了改进型PID计算式
SOLDIER                         2018/01/27     3.0     更改遇限削弱PID算法，针对云台辅助瞄准输入信号复杂，阶跃与
                                                       斜坡信号并存，针对底盘功率限制可能存在较长
													                             时间静差导致速度环崩溃，缩小积分范围。
													                             更改抗积分饱和算法，抑制拨弹电机几个周期出现超调的情况。
SOLDIER                         2018/03/26     3.1     加入滤波器，减少MPU6050高频噪声（第三代车取消MPU6050）
***********************************************************************************************************/

#include "rm_algorithm.h"

/*位置式PID算法 u(k)=Kp*E(K)+Ki*sum(E(K))+Kd*(E(K)-E(K-1))*/

/*******************************************************************
函数名称：CalPID(ST_PID *pstPid)
函数功能：普通的PID算法计算PID量
备    注：
********************************************************************/
void CalPID(ST_PID *pstPid)
{
    pstPid->fpE = pstPid->fpDes - pstPid->fpFB;//计算当前偏差
    if(fabs(pstPid->fpE) <= pstPid->fpEMin)//偏差死区限制
    {
        pstPid->fpE = 0;
    }
    pstPid->fpSumE += pstPid->fpE;
    /*位置式PID计算公式*/
    pstPid->fpU = pstPid->fpKp * pstPid->fpE
                  + pstPid->fpKi * pstPid->fpSumE
                  + pstPid->fpKd * (pstPid->fpE - pstPid->fpPreE);
    pstPid->fpPreE = pstPid->fpE;//保存本次偏差
    /*PID运算输出限幅*/
    if(pstPid->fpU > pstPid->fpUMax)
    {
        pstPid->fpU = pstPid->fpUMax;
    }
    else if(pstPid->fpU < -pstPid->fpUMax)
    {
        pstPid->fpU = -pstPid->fpUMax;
    }

}

/*******************************************************************
函数名称：CalISeparatedPID(ST_PID *pstPid)
函数功能：积分分离式PID算法计算PID量
备    注：积分分离式PID改进算法可减小启动、停止或大幅度增减时较大偏差
          对积分项的积累，从而避免出现较大的超调及振荡现象。
********************************************************************/
void CalISeparatedPID(ST_PID *pstPid)
{
    UCHAR8 uck=1;

    pstPid->fpE=pstPid->fpDes-pstPid->fpFB;//计算当前偏差
    if(fabs(pstPid->fpE) <= pstPid->fpEMin)//偏差死区限制
    {
        pstPid->fpE = 0;
    }
    pstPid->fpSumE += pstPid->fpE;//计算偏差累积
    /*若偏差过大，则积分项不累积偏差*/
    if(fabs(pstPid->fpE) > pstPid->fpEiMax )//判断是否满足积分分离
    {
        uck=0;
    }
    /*位置式PID计算公式*/
    pstPid->fpU = pstPid->fpKp * pstPid->fpE
                  + pstPid->fpKi * pstPid->fpSumE * uck
                  + pstPid->fpKd * (pstPid->fpE - pstPid->fpPreE);
    pstPid->fpPreE = pstPid->fpE;//保存本次偏差
    /*PID运算输出限幅*/
    if(pstPid->fpU > pstPid->fpUMax)
    {
        pstPid->fpU = pstPid->fpUMax;
    }
    else if(pstPid->fpU < -pstPid->fpUMax)
    {
        pstPid->fpU = -pstPid->fpUMax;
    }
}

/*******************************************************************
函数名称：CalIResistedPID(ST_PID *pstPid)
函数功能：抗积分饱和PID算法
备    注：系统往一个方向运动会产生较大积分误差，会在几个周期内产生振荡或超调
********************************************************************/
void CalIResistedPID(ST_PID *pstPid)
{

    pstPid->fpE=pstPid->fpDes-pstPid->fpFB;   //计算当前偏差
    pstPid->fpSumE += pstPid->fpE;   //计算偏差累积

    pstPid->fpSumE = Clip(pstPid->fpSumE, -pstPid->fpEiMax, pstPid->fpEiMax);
    pstPid->fpUi = pstPid->fpKi * pstPid->fpSumE;

    pstPid->fpUp = Clip(pstPid->fpKp * pstPid->fpE, -pstPid->fpEpMax, pstPid->fpEpMax);
    pstPid->fpUd = Clip(pstPid->fpKd * (pstPid->fpE - pstPid->fpPreE), -pstPid->fpEdMax, pstPid->fpEdMax);


    /*若偏差在死区之内，则清零积分累计项*/
    if(fabs(pstPid->fpE) < pstPid->fpEMin)   //判断是否满足积分饱和条件
    {
        pstPid->fpSumE = 0;   //清除偏差累积
    }
    /*位置式PID计算公式*/
    pstPid->fpU = pstPid->fpUp + pstPid->fpUi + pstPid->fpUd;

    pstPid->fpPreE = pstPid->fpE;//保存本次偏差
    /*PID运算输出限幅*/
    pstPid->fpU = Clip(pstPid->fpU, -pstPid->fpUMax, pstPid->fpUMax);
}

/*******************************************************************
函数名称：CalIWeakenPID(ST_PID *pstPid)
函数功能：遇限削弱积分PID改进算法计算PID量
备    注：
********************************************************************/
inline void CalIWeakenPID(ST_PID *pstPid)
{
    pstPid->fpE=pstPid->fpDes-pstPid->fpFB;//计算当前偏差

    if(fabs(pstPid->fpE) < pstPid->fpEMin)
    {
        pstPid->fpSumE += pstPid->fpE;//计算偏差累积
    }

    pstPid->fpSumE = Clip(pstPid->fpSumE, -pstPid->fpEiMax, pstPid->fpEiMax);
    pstPid->fpUi = pstPid->fpKi * pstPid->fpSumE;

    pstPid->fpUp = Clip(pstPid->fpKp * pstPid->fpE, -pstPid->fpEpMax, pstPid->fpEpMax);
    pstPid->fpUd = Clip(pstPid->fpKd * (pstPid->fpE - pstPid->fpPreE), -pstPid->fpEdMax, pstPid->fpEdMax);

    /*位置式PID计算公式*/
    pstPid->fpU = pstPid->fpUp + pstPid->fpUi + pstPid->fpUd;

    pstPid->fpPreE = pstPid->fpE;//保存本次偏差

    /*PID运算输出限幅*/
    pstPid->fpU = Clip(pstPid->fpU, -pstPid->fpUMax, pstPid->fpUMax);
}

/*--------------------------------------------------------------------------------------------------
函数名称：Clip()
函数功能：削波函数，去除超出最大值与最小值之间的值，代之以最大或最小值
--------------------------------------------------------------------------------------------------*/
FP32 Clip(FP32 fpValue, FP32 fpMin, FP32 fpMax)
{
    if(fpValue <= fpMin)
    {
        return fpMin;
    }
    else if(fpValue >= fpMax)
    {
        return fpMax;
    }
    else
    {
        return fpValue;
    }
}

/*******************************************************************
函数名称：Round()
函数功能：将浮点数四舍五入，返回32位整型数
********************************************************************/
SSHORT16 Round(FP32 fp_Round_Number)
{
    if (fp_Round_Number >= 0)
    {
        return (SSHORT16)(fp_Round_Number + 0.5f);
    }
    else
    {
        return (SSHORT16)(fp_Round_Number - 0.5f);
    }
}

/*******************************************************************
函数名称：Sign_Judge(FP32 fp_Any_Number)
函数功能：判断正负
备    注：返回值为1和-1，来改变数的符号
********************************************************************/
SINT32 Sign_Judge(FP32 fp_Judge_Number)
{
    if(fp_Judge_Number > 0)
        return 1;
    else if(fp_Judge_Number < 0)
        return -1;
    else
        return 0;
}

bool RampSignal(float* p_Output, float DesValue, float Step)    //传入指针量向目标值按Step步进，最终稳在DesValue
{
    if(fabs(*p_Output-DesValue) <= Step)
    {
        *p_Output = DesValue;
        return TRUE;
    }
    else
    {
        if(*p_Output < DesValue)
            *p_Output += Step;
        else
            *p_Output -= Step;

        return FALSE;
    }
}

bool RampSignal_s16(s16* p_Output, s16 DesValue, s16 Step)  //传入指针量向目标值按Step步进，最终稳在DesValue
{
    if(abs(*p_Output-DesValue) <= Step)
    {
        *p_Output = DesValue;
        return TRUE;
    }
    else
    {
        if(*p_Output < DesValue)
            *p_Output += Step;
        else
            *p_Output -= Step;

        return FALSE;
    }
}

void TD_Function(TD *ptd)
{
    /*两种方法都可以用，第二种相对而言曲线更加平缓*/
//	int fh=0;
//	float d,a0,y,a1,a2,sa,sy,a=0;
//	ptd->x = ptd->x1-ptd->aim;
//	d = ptd->r*ptd->h*ptd->h;	a0=ptd->h*ptd->x2;	y = ptd->x + a0;	a1 = sqrt(d*d+8*d*abs(y));
//	a2 = a0 + SIGN(y)*(a1-d)/2;
//	sy = (SIGN(y+d)-SIGN(y-d))/2;
//	a  = (a0+y-a2)*sy+a2;
//	sa = (SIGN(a+d)-SIGN(a-d))/2;
//	fh = -ptd->r*(a/d-SIGN(a))*sa - ptd->r*SIGN(a);
//	ptd->x1 +=  0.001*ptd->x2;
//	ptd->x2 +=  0.001*fh;
    /***/
    float d,d0,y,a0,a=0;
    ptd->x = ptd->x1 - ptd->aim;
    d = ptd->r*ptd->h;
    d0=ptd->h * d;
    y = ptd->x + ptd->h*ptd->x2;
    a0 = sqrt(d*d+8*ptd->r*fabs(y));

    if(fabs(y)>d0)
        a = ptd->x2+(a0-d)*Sign_Judge(y)/2;
    else
        a = ptd->x2 + y/ptd->h;

    if(fabs(a)>d)
        y=-1*ptd->r*Sign_Judge(a);
    else
        y=-1*ptd->r*a/d;

    ptd->x1 +=  0.001f*ptd->x2;
    ptd->x2 +=  0.001f*y;
}
