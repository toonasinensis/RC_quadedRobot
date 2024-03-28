#ifndef __NAVIGATION_TASK_H__
#define __NAVIGATION_TASK_H__

#include "hit_global_type.h"
#include "chassis.h"
#include "navigation_algorithm.h"
#include "read_remote_ctrl_task.h"
#include "path_algorithm.h"
#include "system_monitor.h"
//#include "motor_control_task.h"
//#include "remote_control.h"

#define COM_LENGTH 14
//#define _PS_ //光电门的宏
//必须在启用光电门的宏的前提下启用光电门永久宏
#ifdef _PS_
//#define _PS_ALL_TIME //光电门永久宏
#endif

//extern u8 flag_tuoluo;

extern void navigation(void);
extern void PM_navigation(void);

extern cNav nav;
//extern float calibration_current;

//extern bool with_slip_ring_flag;

extern bool flag_reset_q;
extern bool flag_auto_path_q;
extern bool hit_vision_flag;

extern float des[COM_LENGTH];

//extern u8 FLAG_INIT_U, FLAG_INIT_LD, FLAG_INIT_RD;

#endif
