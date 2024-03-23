/**********************************************************************************************************************************************************
版权声明：HITCRT(哈工大竞技机器人协会)
文件名：PID_Algorithm.c
最近修改日期：2016.10.13
版本：1.0
----------------------------------------------------------------------------------------------------------------------------------------------------------
模块描述：
函数列表：
----------------------------------------------------------------------------------------------------------------------------------------------------------
修订记录：
	 作者        	时间            版本     	说明
  pyx & Chris    2016.10.13      	1.0      划分建立此模块
**********************************************************************************************************************************************************/

#include "pid_algorithm.h"

/*以下为位置型PID*/
/*******************************************************************
函数名称：CalPID(ST_PID *this)
函数功能：普通的PID算法计算PID量
备    注：
********************************************************************/
void cPID::CalPID(void)
{
	this->fpE = this->fpDes - this->fpFB; //计算当前偏差
	if (fabs(this->fpE) <= this->fpEMin)  //偏差死区限制
	{
		this->fpE = 0;
		this->fpSumE = 0;
	}
	this->fpSumE += this->fpE;
	/*位置式PID计算公式*/
	this->fpU = this->fpKp * this->fpE + this->fpKi * this->fpSumE + this->fpKd * (this->fpE - this->fpPreE) / 0.002f;
	this->fpPreE = this->fpE; //保存本次偏差
	/*PID运算输出限幅*/
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
函数名称：CalISeparatedPID(ST_PID *this)
函数功能：积分分离式PID算法计算PID量
备    注：积分分离式PID改进算法可减小启动、停止或大幅度增减时较大偏差
		  对积分项的积累，从而避免出现较大的超调及振荡现象。
********************************************************************/
void cPID::CalISeparatedPID(void)
{
	uint8_t uck = 1;

	this->fpE = this->fpDes - this->fpFB; //计算当前偏差
	if (fabs(this->fpE) <= this->fpEMin)  //偏差死区限制
	{
		this->fpE = 0;
	}
	this->fpSumE += this->fpE; //计算偏差累积
	/*若偏差过大，则积分项不累积偏差*/
	if (fabs(this->fpE) > this->fpEiMax) //判断是否满足积分分离
	{
		this->fpSumE = 0;
		uck = 0;
	}
	/*位置式PID计算公式*/
	this->fpU = this->fpKp * this->fpE + this->fpKi * this->fpSumE * uck + this->fpKd * (this->fpE - this->fpPreE);
	this->fpPreE = this->fpE; //保存本次偏差
	/*PID运算输出限幅*/
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
函数名称：CalIResistedPID(ST_PID *this)
函数功能：抗积分饱和PID算法
备    注：系统往一个方向运动会产生较大积分误差，会在几个周期内产生振荡或超调
********************************************************************/
void cPID::CalIResistedPID(void)
{

	this->fpE = this->fpDes - this->fpFB; //计算当前偏差
	this->fpSumE += this->fpE;			  //计算偏差累积

	this->fpSumE = Clip(this->fpSumE, -this->fpEiMax, this->fpEiMax);
	this->fpUi = this->fpKi * this->fpSumE;

	this->fpUp = Clip(this->fpKp * this->fpE, -this->fpEpMax, this->fpEpMax);
	this->fpUd = Clip(this->fpKd * (this->fpE - this->fpPreE), -this->fpEdMax, this->fpEdMax);

	/*若偏差在死区之内，则清零积分累计项*/
	if (fabs(this->fpE) < this->fpEMin) //判断是否满足积分饱和条件
	{
		this->fpSumE = 0; //清除偏差累积
	}
	/*位置式PID计算公式*/
	this->fpU = this->fpUp + this->fpUi + this->fpUd;

	this->fpPreE = this->fpE; //保存本次偏差
							  /*PID运算输出限幅*/
	this->fpU = Clip(this->fpU, -this->fpUMax, this->fpUMax);
}
/*******************************************************************
函数名称：CalIWeakenPID(ST_PID *this)
函数功能：遇限削弱积分PID改进算法计算PID量
备    注：
********************************************************************/
void cPID::CalIWeakenPID(void)
{

	this->fpE = this->fpDes - this->fpFB; //计算当前偏差

	if (((this->fpU <= this->fpUMax && this->fpE > 0) || (this->fpU >= -this->fpUMax && this->fpE < 0)) && fabs(this->fpE) < this->fpEMin)
	{
		this->fpSumE += this->fpE; //计算偏差累积
	}

	this->fpSumE = Clip(this->fpSumE, -this->fpEiMax, this->fpEiMax);
	this->fpUi = this->fpKi * this->fpSumE;

	this->fpUp = Clip(this->fpKp * this->fpE, -this->fpEpMax, this->fpEpMax);
	this->fpUd = Clip(this->fpKd * (this->fpE - this->fpPreE), -this->fpEdMax, this->fpEdMax);

	/*位置式PID计算公式*/
	this->fpU = this->fpUp + this->fpUi + this->fpUd;

	this->fpPreE = this->fpE; //保存本次偏差

	/*PID运算输出限幅*/
	this->fpU = Clip(this->fpU, -this->fpUMax, this->fpUMax);
}

/*******************************************************************
函数名称：CalFilterPID(ST_PID *this)
函数功能：微分项PID输出
备    注：
********************************************************************/
void cPID::CalFilterPID(void)
{
	//=======计算当前偏差===========
	this->fpE = this->fpDes - this->fpFB;
	//=======偏差死区限制========
	if (fabs(this->fpE) <= this->fpEMin)
	{
		this->fpE = 0.0f;
		this->fpUi = 0;
	}
	/*======位置式PID计算公式======*/
	this->fpUp = this->fpKp * this->fpE;			   //比例项输出
	this->fpUi += this->fpKi * this->fpE * this->fpTs; //积分项输出
	//微分项输出及微分滤波
	this->fpUd = this->fpKd * (this->fpE - this->fpPreE) / this->fpTs;

	//===========更新上次偏差===========
	this->fpPreE = this->fpE;
	//===========PID总输出===============
	this->fpU = this->fpUp + this->fpUi + this->fpUd;
	/*=========输出限幅============*/
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
//函数名称：CalComprehensivePID(ST_PID *this)
//函数功能：综合PID输出
//备    注：为调试所用，可参照以上PID进行综合，功能全面
//********************************************************************/
//u8 kkk = 2;

//void cPID::CalComprehensivePID(void)
//{ 
//	uint8_t uck = 0;
//	
//	this->fpE = this->fpDes - this->fpFB;//计算当前偏差
//	this->fpSumE += this->fpE;
//	/*积分抗饱和*/
//	this->fpSumE = ClipFloat(this->fpSumE, -this->fpSumEMax, this->fpSumEMax);
//	/*三个输出项限幅*/
//	this->fpUp = ClipFloat(this->fpKp * this->fpE, -this->fpEpMax, this->fpEpMax);
//	this->fpUi = ClipFloat(this->fpKi * this->fpSumE, -this->fpEiMax, this->fpEiMax);
//	this->fpUd = ClipFloat(this->fpKd * (this->fpE - this->fpPreE), -this->fpEdMax, this->fpEdMax);
//	/*积分分离*/
//	if(fabs(this->fpE) >= this->fpEMax )//判断是否满足积分分离
//	{
//		uck = 0;	
//	}
//	else
//		uck = 1;
//	/*位置式PID计算公式*/
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

//	this->fpPreE = this->fpE;//保存本次偏差
//  /*PID运算输出限幅*/
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
void cTr::LagCompensator(fp32 gain, fp32 t1, fp32 t2) //滞后校正，t2为控制周期取
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
