#include "global_declare.h"

SYSTEM_MONITOR system_monitor = {0};    //系统监视器

/*舵机*/
#if defined Infantry_A
u32 SteeringEnginePWM_High = 500;
u32 SteeringEnginePWM_Low  = 1000;
#elif defined Infantry_B
u32 SteeringEnginePWM_High = 500;
u32 SteeringEnginePWM_Low  = 1009;
#elif defined Infantry_C
u32 SteeringEnginePWM_High = 500;
u32 SteeringEnginePWM_Low  = 1000;
#elif defined Infantry_E
u32 SteeringEnginePWM_High = 250;
u32 SteeringEnginePWM_Low  = 900;
#elif defined robocon_r2
u32 SteeringEnginePWM_High = 500;
u32 SteeringEnginePWM_Low  = 1000;
#else
#error "No defined SelfType"
#endif

IMU_MODE imu_mode = INIT;
