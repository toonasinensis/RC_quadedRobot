#ifndef __FRICTIONWHEEL_TASK_H__
#define __FRICTIONWHEEL_TASK_H__

#include "main.h"


#define Fric 139.0f         //摩擦补偿


typedef enum
{
    FrictionSpeed_Zero   = 1,
    FrictionSpeed_High   = 3,
    FrictionSpeed_Lowest = 5,
    FrictionSpeed_Low    = 7,
} FrictionSpeed;

//滑模结构体
typedef struct
{
    //state
    float fpDes;
    float fpFB;
    float fpE;
    float fpU;
    float fpUMax;

    //para
    float b;        //惯量倒数
    float eps;      //扰动补偿
    float gain;     //比例项
    float dead;     //死区
    TD SmcTd;
} ST_SMC;

extern ST_SMC smc1;
extern ST_SMC smc2;

extern float CM1_TEMP;
extern float CM2_TEMP;

extern u8 State_Flag;
extern float SMC_J;
extern float SMC_f;
extern u32 Inc_time;
extern u32 Dec_time;
extern float Real_Temp1;
extern float Real_Temp2;

void FrictionWheelControl(void);
float SatFunc(float in, float d);
void SlidingModeCtrler(ST_SMC* pst_Smc);
void ShootMode_Select(void);
void intRampSignal(volatile u32* p_Output, u32 DesValue, u32 Step);
void FrictionWheelRevise(void);

#endif
