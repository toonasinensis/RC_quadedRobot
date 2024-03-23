/**********************************************************************************************************************************************************
��Ȩ������HITCRT(�����󾺼�������Э��)
�ļ�����PID_Algorithm.c
����޸����ڣ�2016.10.13
�汾��1.0
----------------------------------------------------------------------------------------------------------------------------------------------------------
ģ��������
�����б�
----------------------------------------------------------------------------------------------------------------------------------------------------------
�޶���¼��
	 ����        	ʱ��            �汾     	˵��
  pyx & Chris    2016.10.13      	1.0      ���ֽ�����ģ��
**********************************************************************************************************************************************************/

#include "pid_algorithm.h"

/*����Ϊλ����PID*/
/*******************************************************************
�������ƣ�CalPID(ST_PID *this)
�������ܣ���ͨ��PID�㷨����PID��
��    ע��
********************************************************************/
void cPID::CalPID(void)
{
	this->fpE = this->fpDes - this->fpFB; //���㵱ǰƫ��
	if (fabs(this->fpE) <= this->fpEMin)  //ƫ����������
	{
		this->fpE = 0;
		this->fpSumE = 0;
	}
	this->fpSumE += this->fpE;
	/*λ��ʽPID���㹫ʽ*/
	this->fpU = this->fpKp * this->fpE + this->fpKi * this->fpSumE + this->fpKd * (this->fpE - this->fpPreE) / 0.002f;
	this->fpPreE = this->fpE; //���汾��ƫ��
	/*PID��������޷�*/
	if (this->fpU > this->fpUMax)
	{
		this->fpU = this->fpUMax;
	}
	else if (this->fpU < -this->fpUMax)
	{
		this->fpU = -this->fpUMax;
	}
}

/*******************************************************************
�������ƣ�CalISeparatedPID(ST_PID *this)
�������ܣ����ַ���ʽPID�㷨����PID��
��    ע�����ַ���ʽPID�Ľ��㷨�ɼ�С������ֹͣ����������ʱ�ϴ�ƫ��
		  �Ի�����Ļ��ۣ��Ӷ�������ֽϴ�ĳ�����������
********************************************************************/
void cPID::CalISeparatedPID(void)
{
	uint8_t uck = 1;

	this->fpE = this->fpDes - this->fpFB; //���㵱ǰƫ��
	if (fabs(this->fpE) <= this->fpEMin)  //ƫ����������
	{
		this->fpE = 0;
	}
	this->fpSumE += this->fpE; //����ƫ���ۻ�
	/*��ƫ������������ۻ�ƫ��*/
	if (fabs(this->fpE) > this->fpEiMax) //�ж��Ƿ�������ַ���
	{
		this->fpSumE = 0;
		uck = 0;
	}
	/*λ��ʽPID���㹫ʽ*/
	this->fpU = this->fpKp * this->fpE + this->fpKi * this->fpSumE * uck + this->fpKd * (this->fpE - this->fpPreE);
	this->fpPreE = this->fpE; //���汾��ƫ��
	/*PID��������޷�*/
	if (this->fpU > this->fpUMax)
	{
		this->fpU = this->fpUMax;
	}
	else if (this->fpU < -this->fpUMax)
	{
		this->fpU = -this->fpUMax;
	}
}

/*******************************************************************
�������ƣ�CalIResistedPID(ST_PID *this)
�������ܣ������ֱ���PID�㷨
��    ע��ϵͳ��һ�������˶�������ϴ���������ڼ��������ڲ����񵴻򳬵�
********************************************************************/
void cPID::CalIResistedPID(void)
{

	this->fpE = this->fpDes - this->fpFB; //���㵱ǰƫ��
	this->fpSumE += this->fpE;			  //����ƫ���ۻ�

	this->fpSumE = Clip(this->fpSumE, -this->fpEiMax, this->fpEiMax);
	this->fpUi = this->fpKi * this->fpSumE;

	this->fpUp = Clip(this->fpKp * this->fpE, -this->fpEpMax, this->fpEpMax);
	this->fpUd = Clip(this->fpKd * (this->fpE - this->fpPreE), -this->fpEdMax, this->fpEdMax);

	/*��ƫ��������֮�ڣ�����������ۼ���*/
	if (fabs(this->fpE) < this->fpEMin) //�ж��Ƿ�������ֱ�������
	{
		this->fpSumE = 0; //���ƫ���ۻ�
	}
	/*λ��ʽPID���㹫ʽ*/
	this->fpU = this->fpUp + this->fpUi + this->fpUd;

	this->fpPreE = this->fpE; //���汾��ƫ��
							  /*PID��������޷�*/
	this->fpU = Clip(this->fpU, -this->fpUMax, this->fpUMax);
}
/*******************************************************************
�������ƣ�CalIWeakenPID(ST_PID *this)
�������ܣ�������������PID�Ľ��㷨����PID��
��    ע��
********************************************************************/
void cPID::CalIWeakenPID(void)
{

	this->fpE = this->fpDes - this->fpFB; //���㵱ǰƫ��

	if (((this->fpU <= this->fpUMax && this->fpE > 0) || (this->fpU >= -this->fpUMax && this->fpE < 0)) && fabs(this->fpE) < this->fpEMin)
	{
		this->fpSumE += this->fpE; //����ƫ���ۻ�
	}

	this->fpSumE = Clip(this->fpSumE, -this->fpEiMax, this->fpEiMax);
	this->fpUi = this->fpKi * this->fpSumE;

	this->fpUp = Clip(this->fpKp * this->fpE, -this->fpEpMax, this->fpEpMax);
	this->fpUd = Clip(this->fpKd * (this->fpE - this->fpPreE), -this->fpEdMax, this->fpEdMax);

	/*λ��ʽPID���㹫ʽ*/
	this->fpU = this->fpUp + this->fpUi + this->fpUd;

	this->fpPreE = this->fpE; //���汾��ƫ��

	/*PID��������޷�*/
	this->fpU = Clip(this->fpU, -this->fpUMax, this->fpUMax);
}

/*******************************************************************
�������ƣ�CalFilterPID(ST_PID *this)
�������ܣ�΢����PID���
��    ע��
********************************************************************/
void cPID::CalFilterPID(void)
{
	//=======���㵱ǰƫ��===========
	this->fpE = this->fpDes - this->fpFB;
	//=======ƫ����������========
	if (fabs(this->fpE) <= this->fpEMin)
	{
		this->fpE = 0.0f;
		this->fpUi = 0;
	}
	/*======λ��ʽPID���㹫ʽ======*/
	this->fpUp = this->fpKp * this->fpE;			   //���������
	this->fpUi += this->fpKi * this->fpE * this->fpTs; //���������
	//΢���������΢���˲�
	this->fpUd = this->fpKd * (this->fpE - this->fpPreE) / this->fpTs;

	//===========�����ϴ�ƫ��===========
	this->fpPreE = this->fpE;
	//===========PID�����===============
	this->fpU = this->fpUp + this->fpUi + this->fpUd;
	/*=========����޷�============*/
	if (this->fpU > this->fpUMax)
	{
		this->fpU = this->fpUMax;
	}
	if (this->fpU < -this->fpUMax)
	{
		this->fpU = -this->fpUMax;
	}
}

///*******************************************************************
//�������ƣ�CalComprehensivePID(ST_PID *this)
//�������ܣ��ۺ�PID���
//��    ע��Ϊ�������ã��ɲ�������PID�����ۺϣ�����ȫ��
//********************************************************************/
//u8 kkk = 2;

//void cPID::CalComprehensivePID(void)
//{ 
//	uint8_t uck = 0;
//	
//	this->fpE = this->fpDes - this->fpFB;//���㵱ǰƫ��
//	this->fpSumE += this->fpE;
//	/*���ֿ�����*/
//	this->fpSumE = ClipFloat(this->fpSumE, -this->fpSumEMax, this->fpSumEMax);
//	/*����������޷�*/
//	this->fpUp = ClipFloat(this->fpKp * this->fpE, -this->fpEpMax, this->fpEpMax);
//	this->fpUi = ClipFloat(this->fpKi * this->fpSumE, -this->fpEiMax, this->fpEiMax);
//	this->fpUd = ClipFloat(this->fpKd * (this->fpE - this->fpPreE), -this->fpEdMax, this->fpEdMax);
//	/*���ַ���*/
//	if(fabs(this->fpE) >= this->fpEMax )//�ж��Ƿ�������ַ���
//	{
//		uck = 0;	
//	}
//	else
//		uck = 1;
//	/*λ��ʽPID���㹫ʽ*/
//	if(this->fpUd > 0.1f)
//	{ 
//		lpf_PID.in = this->fpUd;
//	}
//	
//	LpFilter(&lpf_PID);
//	
//	if(fabs(this->fpE) <= this->fpEMin)
//	{
//	  this->fpU = this->fpUp + kkk * this->fpUi * uck + lpf_PID.out;
//	}
//	else
//	{
//		this->fpU = this->fpUp + this->fpUi * uck + lpf_PID.out;
//	}

//	this->fpPreE = this->fpE;//���汾��ƫ��
//  /*PID��������޷�*/
//  this->fpU = ClipFloat(this->fpU, -this->fpUMax, this->fpUMax);
//}

void cTr::TrF1(fp32 t1, fp32 t2)
{

	this->fpOutput1 = ((2 * t1 + this->fpTs) * this->fpInput1 + (this->fpTs - 2 * t1) * this->fpInputpre1 - (this->fpTs - 2 * t2) * this->fpOutputpre1) / (2 * t2 + this->fpTs);
	this->fpInputpre1 = this->fpInput1;
	this->fpOutputpre1 = this->fpOutput1;
}

void cTr::TrF2(fp32 t)
{

	this->fpOutput2 = 2 * this->fpInput2 - 2 * this->fpInputpre2 - (this->fpTs - 2 * t) * this->fpOutputpre2;
	this->fpInputpre2 = this->fpInput2;
	this->fpOutputpre2 = this->fpOutput2;
}

void cTr::TrF3(fp32 t)
{
	this->fpOutput3 = this->fpInput3 - this->fpInputpre3 - (this->fpTs - 2 * t) * this->fpOutputpre3;
	this->fpInputpre3 = this->fpInput3;
	this->fpOutputpre3 = this->fpOutput3;
}
void cTr::LagCompensator(fp32 gain, fp32 t1, fp32 t2) //�ͺ�У����t2Ϊ��������ȡ
{
	this->fpOutput1 = ((2 * t1 + t2) * this->fpInput1 + (t2 - 2 * t1) * this->fpInputpre1 + (2 * gain * t1 - t2) * this->fpOutputpre1) / (2 * t1 * gain + t2);
	this->fpInputpre1 = this->fpInput1;
	this->fpOutputpre1 = this->fpOutput1;
}

int32_t Sign_Judge(float fp_Judge_Number)
{
	return fp_Judge_Number >= 0 ? 1 : -1;
}

float SatFunc(float in, float d)
{
	if (fabs(in) >= d)
		return Sign_Judge(in);
	else
		return in / d;
}

void TD_Function(TD *ptd)
{
	float d, d0, y, a0, a = 0;
	ptd->x = ptd->x1 - ptd->aim;
	d = ptd->r * ptd->h;
	d0 = ptd->h * d;
	y = ptd->x + ptd->h * ptd->x2;
	a0 = sqrt(d * d + 8 * ptd->r * fabs(y));

	if (fabs(y) > d0)
		a = ptd->x2 + (a0 - d) * Sign_Judge(y) / 2;
	else
		a = ptd->x2 + y / ptd->h;

	if (fabs(a) > d)
		y = -1 * ptd->r * Sign_Judge(a);
	else
		y = -1 * ptd->r * a / d;

	ptd->x1 += ptd->T * ptd->x2;
	ptd->x2 += ptd->T * y;
}

void Clip_TD_Function(TD *pstTd, fp32 lim_x2)
{
	float d, d0, y, a0, a = 0;
	pstTd->x = pstTd->x1 - pstTd->aim;
	d = pstTd->r * pstTd->h;
	d0 = pstTd->h * d;
	y = pstTd->x + pstTd->h * pstTd->x2;
	a0 = sqrt(d * d + 8 * pstTd->r * fabs(y));

	if (fabs(y) > d0)
		a = pstTd->x2 + (a0 - d) * Sign_Judge(y) / 2;
	else
		a = pstTd->x2 + y / pstTd->h;

	if (fabs(a) > d)
		y = -1 * pstTd->r * Sign_Judge(a);
	else
		y = -1 * pstTd->r * a / d;
	lim_x2 = fabs(lim_x2);
	if (pstTd->x2 > lim_x2)
	{
		pstTd->x2 = lim_x2;
	}
	else if (pstTd->x2 < -lim_x2)
	{
		pstTd->x2 = -lim_x2;
	}
	pstTd->x1 += pstTd->T * pstTd->x2;
	pstTd->x2 += pstTd->T * y;
	pstTd->x3 = y;
}

void SlidingModeCtrler(ST_SMC *pst_Smc)
{
	pst_Smc->SmcTd.aim = pst_Smc->fpDes;
	TD_Function(&pst_Smc->SmcTd);
	pst_Smc->fpE = pst_Smc->SmcTd.x1 - pst_Smc->fpFB;
	pst_Smc->fpU = 1 / pst_Smc->b * (pst_Smc->SmcTd.x2 + pst_Smc->eps * SatFunc(pst_Smc->fpE, pst_Smc->dead) + pst_Smc->gain * pst_Smc->fpE);
	if (pst_Smc->fpU > pst_Smc->fpUMax)
	{
		pst_Smc->fpU = +pst_Smc->fpUMax;
	}
	if (pst_Smc->fpU < -pst_Smc->fpUMax)
	{
		pst_Smc->fpU = -pst_Smc->fpUMax;
	}
}
