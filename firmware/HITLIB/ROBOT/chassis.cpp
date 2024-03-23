#include "chassis.h"
#include "navigation_algorithm.h"
//C99中允许该初始化方法
//转向电机(dji 2006)的真实电流无意义
cFourMotor chassis_turn(
		/*velt_pid*/  TURN_VELT_KP, 0, 0, TURN_VELT_UMax , 0, 0, 0, 
		/*pos_pid*/   TURN_POS_KP, TURN_POS_KI, TURN_POS_KD, TURN_POS_UMax, 0, 0, TURN_POS_EMin,
		/*encoder*/   TURN_GEAR_RATIO,  ENCODER_NUMBER,
		/*pid_state*/	DOUBLE_LOOP,
	  /*feed_forward_state*/	WITHOUT_FORWARD,
		/*Ts*/ TURN_VELT_Ts
);
cFourMotor chassis_run(
		/*velt_pid*/  RUN_VELT_KP, 0, 0, RUN_VELT_UMax_NEW , 0, 0, 0, 
		/*pos_pid*/   RUN_POS_KP, RUN_POS_KI, RUN_POS_KD, RUN_POS_UMax, 0, 0, 0,
		/*encoder*/   RUN_GEAR_RATIO,  ENCODER_NUMBER,
		/*pid_state*/	VELT_LOOP,
	  /*feed_forward_state*/	WITHOUT_FORWARD,
		/*Ts*/ 			RUN_VELT_Ts
);

cThreeMotor tri_chassis_turn(
		/*velt_pid*/  TURN_VELT_KP, 0, 0, TURN_VELT_UMax , 0, 0, 0, 
		/*pos_pid*/   TURN_POS_KP, TURN_POS_KI, TURN_POS_KD, TURN_POS_UMax, 0, 0, TURN_POS_EMin,
		/*encoder*/   TURN_GEAR_RATIO,  ENCODER_NUMBER,
		/*pid_state*/	DOUBLE_LOOP,
	  /*feed_forward_state*/	WITHOUT_FORWARD,
		/*Ts*/ 			TURN_VELT_Ts
);


cThreeMotor tri_chassis_run(
		/*velt_pid*/  RUN_VELT_KP, 0, 0, RUN_VELT_UMax_NEW , 0, 0, 0, 
		/*pos_pid*/   RUN_POS_KP, RUN_POS_KI, RUN_POS_KD, RUN_POS_UMax, 0, 0, 0,
		/*encoder*/   RUN_GEAR_RATIO,  ENCODER_NUMBER,
		/*pid_state*/	VELT_LOOP,
	  /*feed_forward_state*/	WITHOUT_FORWARD,
		/*Ts*/ RUN_VELT_Ts
);

chassis_run_state_t chassis_run_state = {FORWARD};
tri_chassis_run_state_t tri_chassis_run_state = {FORWARD};

fp32 chassis_run_leftup_velt_des, chassis_run_leftdown_velt_des, chassis_run_rightup_velt_des, chassis_run_rightdown_velt_des;

void disable_chassis(void)
{
	chassis_turn.pid_state = OPEN_LOOP;
	chassis_turn.feed_forward_state = WITHOUT_FORWARD;
	chassis_turn.rightup_motor.velt_pid.fpU= 0;
	chassis_turn.leftup_motor.velt_pid.fpU = 0;
	chassis_turn.leftdown_motor.velt_pid.fpU = 0;
	chassis_turn.rightdown_motor.velt_pid.fpU = 0;

	chassis_run.pid_state = OPEN_LOOP;
	chassis_run.feed_forward_state = WITHOUT_FORWARD;
	chassis_run.rightup_motor.velt_pid.fpU= 0;
	chassis_run.leftup_motor.velt_pid.fpU = 0;
	chassis_run.leftdown_motor.velt_pid.fpU = 0;
	chassis_run.rightdown_motor.velt_pid.fpU = 0;
}

/*rightup:-45,leftup:45,leftdown:-45,rightdown:45*/
void lock_chassis(void)
{
	chassis_turn.pid_state = DOUBLE_LOOP;
	chassis_turn.feed_forward_state = WITHOUT_FORWARD;
	chassis_turn_with_slip_ring(-45, 45, 45, -45);

	chassis_run.pid_state = VELT_LOOP;//无位置反馈值
	chassis_run.feed_forward_state = WITHOUT_FORWARD;
	chassis_run.rightup_motor.velt_pid.fpDes = 0;
	chassis_run.leftup_motor.velt_pid.fpDes = 0;
	chassis_run.leftdown_motor.velt_pid.fpDes = 0;
	chassis_run.rightdown_motor.velt_pid.fpDes = 0;
	
//	chassis_run.pid_state = OPEN_LOOP;
//	chassis_run.feed_forward_state = WITHOUT_FORWARD;
//	chassis_run.rightup_motor.velt_pid.fpU= 0;
//	chassis_run.leftup_motor.velt_pid.fpU = 0;
//	chassis_run.leftdown_motor.velt_pid.fpU = 0;
//	chassis_run.rightdown_motor.velt_pid.fpU = 0;
}

void stop_chassis(void)
{
	chassis_turn.pid_state = DOUBLE_LOOP;
	chassis_turn.feed_forward_state = WITHOUT_FORWARD;
	chassis_turn_with_slip_ring(0, 0, 0, 0);

	chassis_run.pid_state = VELT_LOOP;//无位置反馈值
	chassis_run.feed_forward_state = WITHOUT_FORWARD;
	chassis_run.rightup_motor.velt_pid.fpDes = 0;
	chassis_run.leftup_motor.velt_pid.fpDes = 0;
	chassis_run.leftdown_motor.velt_pid.fpDes = 0;
	chassis_run.rightdown_motor.velt_pid.fpDes = 0;
}

void disable_tri_chassis(void)
{
	tri_chassis_turn.pid_state = OPEN_LOOP;
	tri_chassis_turn.feed_forward_state = WITHOUT_FORWARD;
	tri_chassis_turn.up_motor.velt_pid.fpU = 0;
	tri_chassis_turn.leftdown_motor.velt_pid.fpU = 0;
	tri_chassis_turn.rightdown_motor.velt_pid.fpU = 0;

	tri_chassis_run.pid_state = OPEN_LOOP;
	tri_chassis_run.feed_forward_state = WITHOUT_FORWARD;
	tri_chassis_run.up_motor.velt_pid.fpU = 0;
	tri_chassis_run.leftdown_motor.velt_pid.fpU = 0;
	tri_chassis_run.rightdown_motor.velt_pid.fpU = 0;
}

/*rightup:-45,leftup:45,leftdown:-45,rightdown:45*/

void lock_tri_chassis(void)
{
	tri_chassis_turn.pid_state = DOUBLE_LOOP;
	tri_chassis_turn.feed_forward_state = WITHOUT_FORWARD;
	tri_chassis_turn_with_slip_ring(90, 30, -30);

	tri_chassis_run.pid_state = OPEN_LOOP;
	tri_chassis_run.feed_forward_state = WITHOUT_FORWARD;
	tri_chassis_run.up_motor.velt_pid.fpU = 0;
	tri_chassis_run.leftdown_motor.velt_pid.fpU = 0;
	tri_chassis_run.rightdown_motor.velt_pid.fpU = 0;
}

void lock_tri_chassis_new(void)
{
	tri_chassis_turn.pid_state = DOUBLE_LOOP;
	tri_chassis_turn.feed_forward_state = WITHOUT_FORWARD;
	tri_chassis_turn_with_slip_ring(90, -30, 30);

	tri_chassis_run.pid_state = OPEN_LOOP;
	tri_chassis_run.feed_forward_state = WITHOUT_FORWARD;
	tri_chassis_run.up_motor.velt_pid.fpU = 0;
	tri_chassis_run.leftdown_motor.velt_pid.fpU = 0;
	tri_chassis_run.rightdown_motor.velt_pid.fpU = 0;
}

void stop_tri_chassis(void)
{
	tri_chassis_turn.pid_state = DOUBLE_LOOP;
	tri_chassis_turn.feed_forward_state = WITHOUT_FORWARD;
	tri_chassis_turn_with_slip_ring(0, 0, 0);

	tri_chassis_run.pid_state = VELT_LOOP;//无位置反馈值
	tri_chassis_run.feed_forward_state = WITHOUT_FORWARD;
	tri_chassis_run.up_motor.velt_pid.fpDes = 0;
	tri_chassis_run.leftdown_motor.velt_pid.fpDes = 0;
	tri_chassis_run.rightdown_motor.velt_pid.fpDes = 0;
}

void stop_tri_chassis_anti_dir(void)
{
	tri_chassis_turn.pid_state = DOUBLE_LOOP;
	tri_chassis_turn.feed_forward_state = WITHOUT_FORWARD;
	tri_chassis_turn_with_slip_ring(90, 90, 90);

	tri_chassis_run.pid_state = VELT_LOOP;//无位置反馈值
	tri_chassis_run.feed_forward_state = WITHOUT_FORWARD;
	tri_chassis_run.up_motor.velt_pid.fpDes = 0;
	tri_chassis_run.leftdown_motor.velt_pid.fpDes = 0;
	tri_chassis_run.rightdown_motor.velt_pid.fpDes = 0;
}
