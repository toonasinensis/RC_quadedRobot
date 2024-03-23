#ifndef __FRICTION_WHEEL_H__
#define __FRICTION_WHEEL_H__

#include "stm32f4xx.h"


#define LeftFrictionPWM     TIM1->CCR4
#define RightFrictionPWM    TIM1->CCR3


void TIM1_Configuration(void);


#endif
