#ifndef __RM_ALGORITHM_H__
#define __RM_ALGORITHM_H__

#include "main.h"

#define abs(x) ((x)>0? (x):(-(x)))

void CalPID(ST_PID *pstPid);
void CalISeparatedPID(ST_PID *pstPid);
void CalIResistedPID(ST_PID *pstPid);
void CalIWeakenPID(ST_PID *pstPid);

FP32 Clip(FP32 fpValue, FP32 fpMin, FP32 fpMax);
SSHORT16 Round(FP32 fp_Round_Number);
SINT32 Sign_Judge(FP32 fp_Judge_Number);
void LpFilter(ST_LPF* lpf);
bool RampSignal(float* p_Output, float DesValue, float Step);
bool RampSignal_s16(s16* p_Output, s16 DesValue, s16 Step); //传入指针量向目标值按Step步进，最终稳在DesValue
void TD_Function(TD *ptd);

#endif
