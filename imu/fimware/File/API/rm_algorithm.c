/**********************************************************************************************************
��Ȩ������ RoboMasters(HITCRT_iHiter ս��)
�ļ����� rm_algorithm.c
����޸����ڣ� 2018/03/26
�汾�� 3.1
-----------------------------------------------------------------------------------------------------------
ģ����������ģ�鶨����RM���õ��㷨��
-----------------------------------------------------------------------------------------------------------
�޸ļ�¼��
����                              ʱ��         �汾   ˵��
��������ع�ˡ��ű���������    2016/01/25     1.0     �������ļ�
FengYun                         2017/03/08	   2.0	   ������PID�㷨����Ӧ�µ�PID�ṹ�壬��ԭ������ʽ�㷨��
                                                       Ϊλ��ʽ�������˸Ľ���PID����ʽ
SOLDIER                         2018/01/27     3.0     ������������PID�㷨�������̨������׼�����źŸ��ӣ���Ծ��
                                                       б���źŲ��棬��Ե��̹������ƿ��ܴ��ڽϳ�
													                             ʱ�侲����ٶȻ���������С���ַ�Χ��
													                             ���Ŀ����ֱ����㷨�����Ʋ�������������ڳ��ֳ����������
SOLDIER                         2018/03/26     3.1     �����˲���������MPU6050��Ƶ��������������ȡ��MPU6050��
***********************************************************************************************************/

#include "rm_algorithm.h"

/*λ��ʽPID�㷨 u(k)=Kp*E(K)+Ki*sum(E(K))+Kd*(E(K)-E(K-1))*/

/*******************************************************************
�������ƣ�CalPID(ST_PID *pstPid)
�������ܣ���ͨ��PID�㷨����PID��
��    ע��
********************************************************************/
void CalPID(ST_PID *pstPid)
{
    pstPid->fpE = pstPid->fpDes - pstPid->fpFB;//���㵱ǰƫ��
    if(fabs(pstPid->fpE) <= pstPid->fpEMin)//ƫ����������
    {
        pstPid->fpE = 0;
    }
    pstPid->fpSumE += pstPid->fpE;
    /*λ��ʽPID���㹫ʽ*/
    pstPid->fpU = pstPid->fpKp * pstPid->fpE
                  + pstPid->fpKi * pstPid->fpSumE
                  + pstPid->fpKd * (pstPid->fpE - pstPid->fpPreE);
    pstPid->fpPreE = pstPid->fpE;//���汾��ƫ��
    /*PID��������޷�*/
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
�������ƣ�CalISeparatedPID(ST_PID *pstPid)
�������ܣ����ַ���ʽPID�㷨����PID��
��    ע�����ַ���ʽPID�Ľ��㷨�ɼ�С������ֹͣ����������ʱ�ϴ�ƫ��
          �Ի�����Ļ��ۣ��Ӷ�������ֽϴ�ĳ�����������
********************************************************************/
void CalISeparatedPID(ST_PID *pstPid)
{
    UCHAR8 uck=1;

    pstPid->fpE=pstPid->fpDes-pstPid->fpFB;//���㵱ǰƫ��
    if(fabs(pstPid->fpE) <= pstPid->fpEMin)//ƫ����������
    {
        pstPid->fpE = 0;
    }
    pstPid->fpSumE += pstPid->fpE;//����ƫ���ۻ�
    /*��ƫ������������ۻ�ƫ��*/
    if(fabs(pstPid->fpE) > pstPid->fpEiMax )//�ж��Ƿ�������ַ���
    {
        uck=0;
    }
    /*λ��ʽPID���㹫ʽ*/
    pstPid->fpU = pstPid->fpKp * pstPid->fpE
                  + pstPid->fpKi * pstPid->fpSumE * uck
                  + pstPid->fpKd * (pstPid->fpE - pstPid->fpPreE);
    pstPid->fpPreE = pstPid->fpE;//���汾��ƫ��
    /*PID��������޷�*/
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
�������ƣ�CalIResistedPID(ST_PID *pstPid)
�������ܣ������ֱ���PID�㷨
��    ע��ϵͳ��һ�������˶�������ϴ���������ڼ��������ڲ����񵴻򳬵�
********************************************************************/
void CalIResistedPID(ST_PID *pstPid)
{

    pstPid->fpE=pstPid->fpDes-pstPid->fpFB;   //���㵱ǰƫ��
    pstPid->fpSumE += pstPid->fpE;   //����ƫ���ۻ�

    pstPid->fpSumE = Clip(pstPid->fpSumE, -pstPid->fpEiMax, pstPid->fpEiMax);
    pstPid->fpUi = pstPid->fpKi * pstPid->fpSumE;

    pstPid->fpUp = Clip(pstPid->fpKp * pstPid->fpE, -pstPid->fpEpMax, pstPid->fpEpMax);
    pstPid->fpUd = Clip(pstPid->fpKd * (pstPid->fpE - pstPid->fpPreE), -pstPid->fpEdMax, pstPid->fpEdMax);


    /*��ƫ��������֮�ڣ�����������ۼ���*/
    if(fabs(pstPid->fpE) < pstPid->fpEMin)   //�ж��Ƿ�������ֱ�������
    {
        pstPid->fpSumE = 0;   //���ƫ���ۻ�
    }
    /*λ��ʽPID���㹫ʽ*/
    pstPid->fpU = pstPid->fpUp + pstPid->fpUi + pstPid->fpUd;

    pstPid->fpPreE = pstPid->fpE;//���汾��ƫ��
    /*PID��������޷�*/
    pstPid->fpU = Clip(pstPid->fpU, -pstPid->fpUMax, pstPid->fpUMax);
}

/*******************************************************************
�������ƣ�CalIWeakenPID(ST_PID *pstPid)
�������ܣ�������������PID�Ľ��㷨����PID��
��    ע��
********************************************************************/
inline void CalIWeakenPID(ST_PID *pstPid)
{
    pstPid->fpE=pstPid->fpDes-pstPid->fpFB;//���㵱ǰƫ��

    if(fabs(pstPid->fpE) < pstPid->fpEMin)
    {
        pstPid->fpSumE += pstPid->fpE;//����ƫ���ۻ�
    }

    pstPid->fpSumE = Clip(pstPid->fpSumE, -pstPid->fpEiMax, pstPid->fpEiMax);
    pstPid->fpUi = pstPid->fpKi * pstPid->fpSumE;

    pstPid->fpUp = Clip(pstPid->fpKp * pstPid->fpE, -pstPid->fpEpMax, pstPid->fpEpMax);
    pstPid->fpUd = Clip(pstPid->fpKd * (pstPid->fpE - pstPid->fpPreE), -pstPid->fpEdMax, pstPid->fpEdMax);

    /*λ��ʽPID���㹫ʽ*/
    pstPid->fpU = pstPid->fpUp + pstPid->fpUi + pstPid->fpUd;

    pstPid->fpPreE = pstPid->fpE;//���汾��ƫ��

    /*PID��������޷�*/
    pstPid->fpU = Clip(pstPid->fpU, -pstPid->fpUMax, pstPid->fpUMax);
}

/*--------------------------------------------------------------------------------------------------
�������ƣ�Clip()
�������ܣ�����������ȥ���������ֵ����Сֵ֮���ֵ����֮��������Сֵ
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
�������ƣ�Round()
�������ܣ����������������룬����32λ������
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
�������ƣ�Sign_Judge(FP32 fp_Any_Number)
�������ܣ��ж�����
��    ע������ֵΪ1��-1�����ı����ķ���
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

bool RampSignal(float* p_Output, float DesValue, float Step)    //����ָ������Ŀ��ֵ��Step��������������DesValue
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

bool RampSignal_s16(s16* p_Output, s16 DesValue, s16 Step)  //����ָ������Ŀ��ֵ��Step��������������DesValue
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
    /*���ַ����������ã��ڶ�����Զ������߸���ƽ��*/
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
