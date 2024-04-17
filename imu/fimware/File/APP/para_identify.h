#ifndef __PARA_IDENTIFY_H__
#define __PARA_IDENTIFY_H__

#include "main.h"

void ParameterInit(void);
float SineSignal_Output(float A, float f, float offset, float acc_tim, float dt);
float State_Sampling(u8 clk_div, float dt);
void CalParameter(void);

#define Clockdivide 2
#define Sample_Num 100
#define IDTFY_PITCH

#ifdef IDTFY_YAW
#define Vector_Order 2
#endif

#ifdef IDTFY_PITCH
#define Vector_Order 4
#endif


#endif

