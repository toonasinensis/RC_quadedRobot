#ifndef __PID_ALGORITHM_H__
#define __PID_ALGORITHM_H__

#define SQUARE(x) ((x) * (x))

#include "math_algorithm.h"
#include "hit_global_type.h"
#include "math.h"

//滑模相关
typedef struct
{
	float x1; //
	float x2;
	float x3;
	float x;
	float r;
	float h;
	float T;
	float aim;
} TD;

typedef struct
{
	// state
	float fpDes;
	float fpFB;
	float fpE;
	float fpU;
	float fpUMax;
	// para
	float b;	//惯量倒数
	float eps;	//扰动补偿
	float gain; //比例项
	float dead; //死区
	TD SmcTd;
} ST_SMC;

class cPID
{
public:
	fp32 fpDes; //控制变量目标值
	fp32 fpFB;	//控制变量反馈值

	fp32 fpKp; //比例系数Kp
	fp32 fpKi; //积分系数Ki
	fp32 fpKd; //微分系数Kd

	fp32 fpE;	 //本次偏差
	fp32 fpPreE; //上次偏差
	fp32 fpSumE; //总偏差
	fp32 fpInput;
	fp32 fpInputpre;
	fp32 fpOutput;
	fp32 fpOutputpre;
	fp32 fpEpMax; //比例项输出最大值
	fp32 fpEiMax; //积分项输出最大值
	fp32 fpEdMax; //微分项输出最大值
	fp32 fpEMin;  //积分上限

	fp32 fpUp; //比例输出
	fp32 fpUi; //积分输出
	fp32 fpUd; //微分输出
	fp32 fpU;  //本次PID运算结果
	fp32 fpUpre;
	fp32 fpUMax; // PID运算后输出最大值及做遇限削弱时的上限值
	fp32 fpTs;	 // PID控制周期，单位：s
	cPID() {}
	cPID(fp32 Kp, fp32 Ki, fp32 Kd, fp32 UpMax, fp32 EiMax, fp32 UdMax, fp32 EMin, fp32 ts)
	{
		this->fpDes = 0;
		this->fpFB = 0;

		this->fpKp = Kp;
		this->fpKi = Ki;
		this->fpKd = Kd;

		this->fpUp = 0;
		this->fpUi = 0;
		this->fpUd = 0;

		this->fpE = 0;
		this->fpPreE = 0;
		this->fpSumE = 0;

		this->fpU = 0;
		this->fpUpre = 0;
		this->fpUMax = UpMax;
		this->fpEpMax = UpMax;
		this->fpEiMax = EiMax;
		this->fpEdMax = UdMax;
		this->fpEMin = EMin;
		this->fpTs = ts;
	} // initialize PID
	void CalPID(void);
	void CalISeparatedPID(void);
	void CalIResistedPID(void);
	void CalIWeakenPID(void);
	void CalFilterPID(void);
};
class cTr
{

public:
	fp32 fpInput1;
	fp32 fpInput2;
	fp32 fpInput3;
	fp32 fpInputpre1;
	fp32 fpInputpre2;
	fp32 fpInputpre3;
	fp32 fpOutput1;
	fp32 fpOutput2;
	fp32 fpOutput3;
	fp32 fpOutputpre1;
	fp32 fpOutputpre2;
	fp32 fpOutputpre3;
	fp32 fpTs;

	cTr(fp32 Ts) //采样周期
	{
		this->fpTs = Ts;
	}

	void TrF1(fp32 t1, fp32 t2);
	void TrF2(fp32 t);
	void TrF3(fp32 t);
	void LagCompensator(fp32 gain, fp32 t1, fp32 t2);
};
extern void SlidingModeCtrler(ST_SMC *pst_Smc);
void TD_Function(TD *ptd);
void Clip_TD_Function(TD *pstTd, fp32 lim_x2);

#endif
