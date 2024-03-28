#ifndef __MOTOR_CONTROL_TASK_H__
#define __MOTOR_CONTROL_TASK_H__

#include "chassis.h"
//#include "pid_algorithm.h"
//#include "math_algorithm.h"
//#include "action_task.h"
#include "system_monitor.h"
#include "parallel_mechanism.h"

#define CHASSIS_TURN_LIMIT_ANGLE 105

extern void chassis_control(void);
void ParallelMechanism_control(void);

//extern void tri_chassis_control(void);
//extern void cal_chassis_ramp_des(fp32 run_step, fp32 turn_step);
//extern void ramp_signal(float& Output, float DesValue, float Step);

#endif
