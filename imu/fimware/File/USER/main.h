#ifndef __MAIN_H__
#define __MAIN_H__

#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "hal.h"
#include "gimbal_control_types.h"

#include "global_declare.h"
#include "os.h"
#include "bsp_init.h"
#include "systemmonitor_task.h"
#include "gimbal_task.h"
#include "led.h"
#include "distance_measure_task.h"


#include "imu_update.h"
#include "temp_control.h"
#include "rm_algorithm.h"
#include "verify.h"
#include "dji_protocol.h"
#include "usart_protocol.h"
#include "kalman_filter.h"
#include "general_math.h"
#include "para_identify.h"
#include "gimbal_app.h"
#include "bmi088_driver.h"

int main(void);
void SystemMonitorTask(void);
void IRQHandlerTask(void);
void IMUSampleTask(void);
void IMUUpdateTask(void);
void TempControlTask(void);
void SendDataTask(void);
void GimbalTask(void);
void LedTask(void);
void DebugTask(void);
#endif


//#define __MAIN_H__


///*---------------------------RM_TYPES---------------------------*/
////#include "rm_data_types.h"
////#include "rm_robot_types.h"
////#include "rm_rc_types.h"

///*---------------------------USER-------------------------------*/
//#include "os.h"
//#include "stm32f4xx_it.h"
//#include "global_declare.h"
////#include "globaluse_basic_function.h"

///*---------------------------RM_TASKS---------------------------*/
//#include "systemmonitor_task.h"
//#include "gimbal_task.h"
//#include "supply_pellet_task.h"
//#include "friction_wheel_task.h"

///*---------------------------API--------------------------------*/
//#include "rm_algorithm.h"
////#include "encode_process.h"
//#include "dji_protocol.h"
//#include "usart_protocol.h"
////#include "debug_protocol.h"
//#include "time_estimate.h"
//#include "icm20602_driver.h"
//#include "general_math.h"
//#include "para_identify.h"

///*---------------------------BSP--------------------------------*/
//#include "bsp_init.h"

///*---------------------------STM32LIB---------------------------*/
//#include "stm32f4xx.h"

///*---------------------------C_LIB------------------------------*/
//#include "stdio.h"
//#include "string.h"
//#include "stdlib.h"
//#include "math.h"

///*---------------------------MATH-------------------------------*/
//#include "arm_math.h"

///*---------------------------PERSONAL---------------------------*/



///*---------------------------Function---------------------------*/
//int main(void);
//void SystemMonitorTask(void);
//void IRQHandlerTask(void);
//void IMUSampleTask(void);
//void IMUUpdateTask(void);
//void TempControlTask(void);
//void GimbalTask(void);
//void SupplyPelletTask(void);
//void FrictionWheelTask(void);
//void LedTask(void);
//void DebugTask(void);


//#endif
