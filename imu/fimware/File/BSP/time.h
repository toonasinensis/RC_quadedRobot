#ifndef _TIME_H_
#define _TIME_H_

#include "main.h"

#define Camera_Prescaler 168-1
#define Camera_Period    1000-1
#define Camera_Pause     1

void TIM1_PWM_Init(void);
void TIM2_3_Configuration(void);
void TIM12_PWM_Init(void);



#endif
