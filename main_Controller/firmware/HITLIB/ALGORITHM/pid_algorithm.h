#ifndef __PID_ALGORITHM_H__
#define __PID_ALGORITHM_H__

#define SQUARE(x) ((x) * (x))

#include "math_algorithm.h"
#include "hit_global_type.h"
#include "math.h"

//��ģ���
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
	float b;	//��������
	float eps;	//�Ŷ�����
	float gain; //������
	float dead; //����
	TD SmcTd;
} ST_SMC;

class cPID
{
public:
	fp32 fpDes; //���Ʊ���Ŀ��ֵ
	fp32 fpFB;	//���Ʊ�������ֵ

	fp32 fpKp; //����ϵ��Kp
	fp32 fpKi; //����ϵ��Ki
	fp32 fpKd; //΢��ϵ��Kd

	fp32 fpE;	 //����ƫ��
	fp32 fpPreE; //�ϴ�ƫ��
	fp32 fpSumE; //��ƫ��
	fp32 fpInput;
	fp32 fpInputpre;
	fp32 fpOutput;
	fp32 fpOutputpre;
	fp32 fpEpMax; //������������ֵ
	fp32 fpEiMax; //������������ֵ
	fp32 fpEdMax; //΢����������ֵ
	fp32 fpEMin;  //��������

	fp32 fpUp; //�������
	fp32 fpUi; //�������
	fp32 fpUd; //΢�����
	fp32 fpU;  //����PID������
	fp32 fpUpre;
	fp32 fpUMax; // PID�����������ֵ������������ʱ������ֵ
	fp32 fpTs;	 // PID�������ڣ���λ��s
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

	cTr(fp32 Ts) //��������
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
