#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include "pid_algorithm.h"
#include "motor_drive.h"

// 8矢量底盘需要的尺寸，单位mm
#define L_EIGHT 202.65f // 200.0f//430.0f//200.0f//270.0f//半车长
#define A_EIGHT 202.65f // 280.0f//350.0f//120.0f//230.0f//半车宽
#define R_WHEEL 63.5f//44.0f//适当给小R_WHEEL，这样可以使except_global_velt小于velt_fb，达到想要的效果

//三角底盘尺寸，单位mm
//#define TRI_CHASSIS_L 470.0f//任意两轮系中心的距离

#define TRI_CHASSIS_R 430.0f

#define COS_THETA 0.8660254038f
#define SIN_THETA 0.5f

#define ENCODER_NUMBER 8191
#define RUN_GEAR_RATIO 6.353f			  //行进电机减速比
#define TURN_GEAR_RATIO 36.0f * 86.0f / 18.0f  //2006减速比36，小轮大轮齿比65：18

#define RUN_VELT_KP 	800//450
//750//650//600
#define RUN_VELT_KI 	5//0
#define RUN_VELT_KD 	0
#define RUN_VELT_UMax	25000.0f//45000.0f//45000.0f
#define RUN_VELT_UMax_NEW	50000.0f//45000.0f//45000.0f
#define RUN_VELT_Ts		0.001f


#define RUN_POS_KP 		0
#define RUN_POS_KI  	0
#define RUN_POS_KD  	0
#define RUN_POS_UMax  400.0f	//避免转速过快，电机反电动势超过电源电压
#define RUN_POS_Ts		0.001f

#define TURN_POS_KP   	15.0f
#define TURN_POS_KI  	  0.0f//1.5f
#define TURN_POS_KD   	0.0f
#define TURN_POS_EMin 	0.3f
#define TURN_POS_UMax   100.0f
#define TURN_POS_Ts     0.001f

#define TURN_VELT_KP  	600.0f//300
#define TURN_VELT_KI  	0.0f
#define TURN_VELT_KD  	0.0f
#define TURN_VELT_UMax	10000.0f
#define TURN_VELT_Ts    0.001f

/*feedforward*///24.5072
#define K_UP_STRAIGHT	7.177263990623291f
#define K_RD_STRAIGHT	7.515211984524339f	
#define K_LD_STRAIGHT	7.992413258817520f	

#define START_CURRENT_UP 2.180650738014755e+03
#define START_CURRENT_RD 1.374103328460389e+03
#define START_CURRENT_LD 1.621368729989229e+03f

#define K_UP_TURN		0.0f
#define K_LD_TURN		0.0f
#define K_RD_TURN		0.0f

#define K_LU_STRAIGHT 	0.0f	//14.93947332541534f
#define K_RU_STRAIGHT 	0.0f	//14.93947332541534f
#define K_LU_TURN 		0		
#define K_RU_TURN 		0		

#define START_CURRENT 	1.693297484071623e+03f//3698.3f


typedef enum
{
	VELT_LOOP,
	POS_LOOP,
	DOUBLE_LOOP,
	OPEN_LOOP//开环状态下，电流输出等于电流环输出，但不进行PID计算
}motor_pid_state_e;

typedef enum
{
	WITH_FORWARD,
	WITHOUT_FORWARD
}motor_feed_forward_state_e;

typedef enum
{
	BACKWARD,
	FORWARD
}chassis_run_state_e;
typedef struct
{
	ST_VECTOR rightup, rightdown, leftup, leftdown;
}chassis_velt_t;

typedef struct
{
	ST_VECTOR up, rightdown, leftdown;
}tri_chassis_velt_t;

typedef struct
{
	chassis_run_state_e rightup, rightdown, leftup, leftdown;
}chassis_run_state_t;

typedef struct
{
	chassis_run_state_e up, leftdown, rightdown;
}tri_chassis_run_state_t;

class cFourMotor
{
	public:
	motor_pid_state_e pid_state;
	motor_feed_forward_state_e feed_forward_state;
	cMotor rightup_motor, leftup_motor, leftdown_motor, rightdown_motor;
	cFourMotor(){}
	
	cFourMotor(fp32 vKp, fp32 vKi, fp32 vKd, fp32 vUpMax, fp32 vEiMax, fp32 vUdMax, fp32 vEMin,
						 fp32 pKp, fp32 pKi, fp32 pKd, fp32 pUpMax, fp32 pEiMax, fp32 pUdMax, fp32 pEMin,
				     fp32 gr, int32_t num, motor_pid_state_e ps, motor_feed_forward_state_e ffs,fp32 ts)	
	{
		pid_state = ps;
		feed_forward_state = ffs;
		rightup_motor = cMotor(vKp, vKi, vKd, vUpMax, vEiMax, vUdMax, vEMin,
													 pKp, pKi, pKd, pUpMax, pEiMax, pUdMax, pEMin,
				                   gr, num,ts);
		leftup_motor = rightup_motor;
		leftdown_motor = rightup_motor;
		rightdown_motor = rightup_motor;
	}
};

class cThreeMotor
{
	public:
	ST_VECTOR pos_up;
	ST_VECTOR pos_right;
	ST_VECTOR pos_left;
	
	motor_pid_state_e pid_state;
	motor_feed_forward_state_e feed_forward_state;
	cMotor up_motor, leftdown_motor, rightdown_motor;
	cThreeMotor(){}
	cThreeMotor(fp32 vKp, fp32 vKi, fp32 vKd, fp32 vUpMax, fp32 vEiMax, fp32 vUdMax, fp32 vEMin,
						 fp32 pKp, fp32 pKi, fp32 pKd, fp32 pUpMax, fp32 pEiMax, fp32 pUdMax, fp32 pEMin,
				     fp32 gr, int32_t num, motor_pid_state_e ps, motor_feed_forward_state_e ffs,fp32 ts)	
	{
		pid_state = ps;
		feed_forward_state = ffs;
		up_motor = cMotor(vKp, vKi, vKd, vUpMax, vEiMax, vUdMax, vEMin,
								pKp, pKi, pKd, pUpMax, pEiMax, pUdMax, pEMin, gr, num,ts);
		leftdown_motor = up_motor;
		rightdown_motor = up_motor;
	pos_up.fpVx = 0;
	pos_up.fpVy = TRI_CHASSIS_R;
		
	pos_right.fpVx = TRI_CHASSIS_R*COS_THETA;
	pos_right.fpVy = -TRI_CHASSIS_R*SIN_THETA;
	pos_left.fpVx = -TRI_CHASSIS_R*COS_THETA;
	pos_left.fpVy = -TRI_CHASSIS_R*SIN_THETA;
	}
};


extern void disable_chassis(void);
extern void lock_chassis(void);
extern void stop_chassis(void);

extern void disable_tri_chassis(void);
extern void lock_tri_chassis(void);
extern void lock_tri_chassis_new(void);
extern void stop_tri_chassis(void);
extern void stop_tri_chassis_anti_dir(void);

extern cFourMotor chassis_turn, chassis_run,chassis_absolute_turn;
extern cThreeMotor tri_chassis_turn, tri_chassis_run, tri_chassis_absolute_turn;
extern fp32 chassis_run_leftup_velt_des, chassis_run_leftdown_velt_des, chassis_run_rightup_velt_des, chassis_run_rightdown_velt_des;
extern fp32 chassis_turn_leftup_pos_des, chassis_turn_leftdown_pos_des, chassis_turn_rightup_pos_des, chassis_turn_rightdown_pos_des;
extern chassis_run_state_t chassis_run_state;
extern tri_chassis_run_state_t tri_chassis_run_state;

#endif
