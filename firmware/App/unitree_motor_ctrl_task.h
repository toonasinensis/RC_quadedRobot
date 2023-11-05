#ifndef __UNITREE_MOTOR_CTRL_TASK_H__
#define __UNITREE_MOTOR_CTRL_TASK_H__
//#include "hitcrt_types.h"
#include "unitree_motor.h"
#include "global_declare.h"
#include "usart.h"
//#include "unitree_motor.h"

//#include "motor_infro.h"
//#include "robot.h"
//#include "delay.h"




//宇树A1参数限制
//T < 250NM   W < 500 rad/s  Pos < 130000 rad   KP < 16 KW < 32 
//官方给的，可能太大了

#define UNITREE_MOTOR_MAX_KP 1.0f   
#define UNITREE_MOTOR_MAX_KD 3.5f  //K_W





void receive_motor_feedback(uint8_t *raw_data, leg_t *leg);
void update_command(void);
void send_command(void);
void enable_all_motor(void);
void disable_all_motor(void);

void unitree_motor_crl_task(void);  //接收数据在串口中断，发送在这里
//void LpFilter(ST_LPF *lpf);
void filter_command(void);
void param_limit(void);
void get_init_pos(void);



#endif
