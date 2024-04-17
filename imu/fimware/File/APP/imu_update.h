#ifndef __IMU_UPDATE__
#define __IMU_UPDATE__

#include "main.h"
#include "global_declare.h"
#include "bmi088_driver.h"

void IMU_Update_Mahony(_imu_st *imu,float dt);
extern	_imu_st imu_data;
extern float q0_send,q1_send,q2_send,q3_send;
#endif

