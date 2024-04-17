#ifndef __STEERING_ENGINE_H__
#define __STEERING_ENGINE_H__

#include "stm32f4xx.h"
#include "global_declare.h"

//PB8----TIM4CH3----SteeringEnginePWM1----舵机
//PB9----TIM4CH4----SteeringEnginePWM2----舵机



#define SteeringEnginePWM1 TIM4->CCR3     //舵机，PB8
#define SteeringEnginePWM2 TIM4->CCR4     //舵机，PB9


#define SteeringEngine_Prescaler 168-1
#define SteeringEngine_Period    10000-1

typedef enum
{
    Servo_Close = 0,
    Servo_Open = 1,
} ServoState;

void TIM4_Configuration(void);
void Servo_Control(ServoState state);


#endif
