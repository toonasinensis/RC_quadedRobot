#include "motor_control_task.h"
#include "bsp_fdcan.h"

//一定要在最后一步再转化为16位整型，不然会溢出
const fp32 current_max = 100000.0f;
const fp32 k_zoom = 30000.0f / current_max;
fp32 run_leftup_current, run_leftdown_current, run_rightup_current, run_rightdown_current;

void chassis_control(void)
{
	int16_t turn_leftup_current, turn_leftdown_current, turn_rightup_current, turn_rightdown_current;
//	fp32 run_leftup_current, run_leftdown_current, run_rightup_current, run_rightdown_current;
	static motor_pid_state_e pre_run_pid_state;
	static motor_pid_state_e pre_turn_pid_state;
	const fp32 safe_current_max = 60000.0f; // 60000.0f
	const fp32 run_velt_max = 1000.0f;		// 280.0f;

	if (system_state != SYS_RUN)
	{
		return;
	}

	switch (chassis_run.pid_state)
	{
	case DOUBLE_LOOP:
		chassis_run.rightdown_motor.pos_pid.CalFilterPID();
		chassis_run.rightdown_motor.velt_pid.fpDes = chassis_run.rightdown_motor.pos_pid.fpU;
		chassis_run.rightdown_motor.velt_pid.CalPID();

		chassis_run.rightup_motor.pos_pid.CalFilterPID();
		chassis_run.rightup_motor.velt_pid.fpDes = chassis_run.rightup_motor.pos_pid.fpU;
		chassis_run.rightup_motor.velt_pid.CalPID();

		chassis_run.leftup_motor.pos_pid.CalFilterPID();
		chassis_run.leftup_motor.velt_pid.fpDes = chassis_run.leftup_motor.pos_pid.fpU;
		chassis_run.leftup_motor.velt_pid.CalPID();

		chassis_run.leftdown_motor.pos_pid.CalFilterPID();
		chassis_run.leftdown_motor.velt_pid.fpDes = chassis_run.leftdown_motor.pos_pid.fpU;
		chassis_run.leftdown_motor.velt_pid.CalPID();
		break;
	case POS_LOOP:
		break;
	case VELT_LOOP:
		chassis_run.rightdown_motor.velt_pid.fpDes = ClipFloat(chassis_run.rightdown_motor.velt_pid.fpDes,
															   -run_velt_max, run_velt_max);
		chassis_run.rightup_motor.velt_pid.fpDes = ClipFloat(chassis_run.rightup_motor.velt_pid.fpDes,
															 -run_velt_max, run_velt_max);
		chassis_run.leftdown_motor.velt_pid.fpDes = ClipFloat(chassis_run.leftdown_motor.velt_pid.fpDes,
															  -run_velt_max, run_velt_max);
		chassis_run.leftup_motor.velt_pid.fpDes = ClipFloat(chassis_run.leftup_motor.velt_pid.fpDes,
															-run_velt_max, run_velt_max);
		chassis_run.rightdown_motor.velt_pid.CalFilterPID();
		chassis_run.rightup_motor.velt_pid.CalFilterPID();
		chassis_run.leftdown_motor.velt_pid.CalFilterPID();
		chassis_run.leftup_motor.velt_pid.CalFilterPID();
		break;
	case OPEN_LOOP:
		if (pre_run_pid_state != OPEN_LOOP)
		{
			//	  		chassis_run.leftup_motor.velt_pid.fpU = 0;
			//	  		chassis_run.leftdown_motor.velt_pid.fpU = 0;
			//	  		chassis_run.rightup_motor.velt_pid.fpU = 0;
			//	  		chassis_run.rightdown_motor.velt_pid.fpU = 0;
		}
		break;
	}

	run_leftup_current = chassis_run.leftup_motor.velt_pid.fpU;
	run_leftdown_current = chassis_run.leftdown_motor.velt_pid.fpU;
	run_rightup_current = chassis_run.rightup_motor.velt_pid.fpU;
	run_rightdown_current = chassis_run.rightdown_motor.velt_pid.fpU;

	if (chassis_run.feed_forward_state == WITH_FORWARD)
	{
		run_leftup_current += chassis_run.leftup_motor.feed_forward_current;
		run_leftdown_current += chassis_run.leftdown_motor.feed_forward_current;
		run_rightup_current += chassis_run.rightup_motor.feed_forward_current;
		run_rightdown_current += chassis_run.rightdown_motor.feed_forward_current;
	}

	run_leftup_current = ClipFloat(run_leftup_current, -safe_current_max, safe_current_max);
	run_leftdown_current = ClipFloat(run_leftdown_current, -safe_current_max, safe_current_max);
	run_rightup_current = ClipFloat(run_rightup_current, -safe_current_max, safe_current_max);
	run_rightdown_current = ClipFloat(run_rightdown_current, -safe_current_max, safe_current_max);

	switch (chassis_turn.pid_state)
	{
	case DOUBLE_LOOP:
		chassis_turn.rightdown_motor.pos_pid.CalFilterPID();
		chassis_turn.rightdown_motor.velt_pid.fpDes = chassis_turn.rightdown_motor.pos_pid.fpU;
		chassis_turn.rightdown_motor.velt_pid.CalPID();

		chassis_turn.rightup_motor.pos_pid.CalFilterPID();
		chassis_turn.rightup_motor.velt_pid.fpDes = chassis_turn.rightup_motor.pos_pid.fpU;
		chassis_turn.rightup_motor.velt_pid.CalPID();

		chassis_turn.leftup_motor.pos_pid.CalFilterPID();
		chassis_turn.leftup_motor.velt_pid.fpDes = chassis_turn.leftup_motor.pos_pid.fpU;
		chassis_turn.leftup_motor.velt_pid.CalPID();

		chassis_turn.leftdown_motor.pos_pid.CalFilterPID();
		chassis_turn.leftdown_motor.velt_pid.fpDes = chassis_turn.leftdown_motor.pos_pid.fpU;
		chassis_turn.leftdown_motor.velt_pid.CalPID();
		break;
	case POS_LOOP:
		break;
	case VELT_LOOP:
		break;
	case OPEN_LOOP:
		if (pre_turn_pid_state != OPEN_LOOP)
		{
			chassis_turn.leftup_motor.velt_pid.fpU = 0;
			chassis_turn.leftdown_motor.velt_pid.fpU = 0;
			chassis_turn.rightup_motor.velt_pid.fpU = 0;
			chassis_turn.rightdown_motor.velt_pid.fpU = 0;
		}
		break;
	}

	turn_leftup_current = chassis_turn.leftup_motor.velt_pid.fpU;
	turn_leftdown_current = chassis_turn.leftdown_motor.velt_pid.fpU;
	turn_rightup_current = chassis_turn.rightup_motor.velt_pid.fpU;
	turn_rightdown_current = chassis_turn.rightdown_motor.velt_pid.fpU;

	if (chassis_turn.feed_forward_state == WITH_FORWARD)
	{
		turn_leftup_current += chassis_turn.leftup_motor.feed_forward_current;
		turn_leftdown_current += chassis_turn.leftdown_motor.feed_forward_current;
		turn_rightup_current += chassis_turn.rightup_motor.feed_forward_current;
		turn_rightdown_current += chassis_turn.rightdown_motor.feed_forward_current;
	}
//可以改变电机实际的转动方向
  can1_msg.id = 0x200;
	can1_msg.buffer[0] = ((int16_t)(run_rightup_current * k_zoom))>>8;
	can1_msg.buffer[1] = ((int16_t)(run_rightup_current * k_zoom));
	can1_msg.buffer[2] = ((int16_t)(run_rightdown_current * k_zoom))>>8;
	can1_msg.buffer[3] = ((int16_t)(run_rightdown_current * k_zoom));
	can1_msg.buffer[4] = ((int16_t)(run_leftdown_current * k_zoom))>>8;
	can1_msg.buffer[5] = ((int16_t)(run_leftdown_current * k_zoom));
	can1_msg.buffer[6] = ((int16_t)(run_leftup_current * k_zoom))>>8;
	can1_msg.buffer[7] = ((int16_t)(run_leftup_current * k_zoom));
	
	fdcan1.send_MSG(&can1_msg);
	
	pre_turn_pid_state = chassis_turn.pid_state;
	pre_run_pid_state = chassis_run.pid_state;
}

//extern u8 duty_sucker;
//void tri_chassis_control(void)
//{
//	tri_chassis_type_t run_current, turn_current;

//	fp32 run_up_current, run_leftdown_current, run_rightdown_current;
//	int16_t turn_up_current, turn_leftdown_current, turn_rightdown_current;
//	static motor_pid_state_e pre_turn_pid_state;

//	//一定要在最后一步再转化为16位整型，不然会溢出
//	const fp32 current_max = 100000.0f;
//	const fp32 k_zoom = 30000.0f / current_max;
//	// const fp32 safe_current_max = 30000.0f;//50000.0f;
//	const fp32 new_safe_current_max = 100000.0f; // 60000.0f
//	const fp32 n5055_velt_max = 1000.0f;		 // 280.0f;
//	// fp32 run_leftup_current, run_leftdown_current, run_rightup_current, run_rightdown_current;
//	static motor_pid_state_e pre_run_pid_state;

//	//	if(system_state != SYS_RUN)
//	//	{
//	//		return;
//	//	}

//	/*主动轮*/
//	switch (tri_chassis_run.pid_state)
//	{
//	case DOUBLE_LOOP:
//		tri_chassis_run.up_motor.pos_pid.CalFilterPID();
//		tri_chassis_run.up_motor.velt_pid.fpDes = tri_chassis_run.up_motor.pos_pid.fpU;
//		tri_chassis_run.up_motor.velt_pid.CalPID();

//		tri_chassis_run.leftdown_motor.pos_pid.CalFilterPID();
//		tri_chassis_run.leftdown_motor.velt_pid.fpDes = tri_chassis_run.leftdown_motor.pos_pid.fpU;
//		tri_chassis_run.leftdown_motor.velt_pid.CalPID();

//		tri_chassis_run.rightdown_motor.pos_pid.CalFilterPID();
//		tri_chassis_run.rightdown_motor.velt_pid.fpDes = tri_chassis_run.rightdown_motor.pos_pid.fpU;
//		tri_chassis_run.rightdown_motor.velt_pid.CalPID();
//		break;
//	case POS_LOOP:
//		break;
//	case VELT_LOOP:
//		tri_chassis_run.rightdown_motor.velt_pid.fpDes = ClipFloat(tri_chassis_run.rightdown_motor.velt_pid.fpDes,
//																   -n5055_velt_max, n5055_velt_max);
//		tri_chassis_run.leftdown_motor.velt_pid.fpDes = ClipFloat(tri_chassis_run.leftdown_motor.velt_pid.fpDes,
//																  -n5055_velt_max, n5055_velt_max);
//		tri_chassis_run.up_motor.velt_pid.fpDes = ClipFloat(tri_chassis_run.up_motor.velt_pid.fpDes,
//															-n5055_velt_max, n5055_velt_max);
//		tri_chassis_run.up_motor.velt_pid.CalFilterPID();
//		tri_chassis_run.leftdown_motor.velt_pid.CalFilterPID();
//		tri_chassis_run.rightdown_motor.velt_pid.CalFilterPID();
//		break;
//	case OPEN_LOOP:
//		if (pre_run_pid_state != OPEN_LOOP)
//		{
//			//			tri_chassis_run.up_motor.velt_pid.fpU = 0;
//			//			tri_chassis_run.leftdown_motor.velt_pid.fpU = 0;
//			//			tri_chassis_run.rightdown_motor.velt_pid.fpU = 0;
//		}
//		break;
//	}

//	run_up_current = tri_chassis_run.up_motor.velt_pid.fpU;
//	run_leftdown_current = tri_chassis_run.leftdown_motor.velt_pid.fpU;
//	run_rightdown_current = tri_chassis_run.rightdown_motor.velt_pid.fpU;

//	if (tri_chassis_run.feed_forward_state == WITH_FORWARD)
//	{
//		run_up_current += tri_chassis_run.up_motor.feed_forward_current;
//		run_leftdown_current += tri_chassis_run.leftdown_motor.feed_forward_current;
//		run_rightdown_current += tri_chassis_run.rightdown_motor.feed_forward_current;
//	}

//	run_up_current = ClipFloat(run_up_current, -new_safe_current_max, new_safe_current_max);
//	run_leftdown_current = ClipFloat(run_leftdown_current, -new_safe_current_max, new_safe_current_max);
//	run_rightdown_current = ClipFloat(run_rightdown_current, -new_safe_current_max, new_safe_current_max);

//	switch (tri_chassis_turn.pid_state)
//	{
//	case DOUBLE_LOOP:
//		tri_chassis_turn.up_motor.pos_pid.CalFilterPID();
//		tri_chassis_turn.up_motor.velt_pid.fpDes = tri_chassis_turn.up_motor.pos_pid.fpU;
//		tri_chassis_turn.up_motor.velt_pid.CalPID();

//		tri_chassis_turn.leftdown_motor.pos_pid.CalFilterPID();
//		tri_chassis_turn.leftdown_motor.velt_pid.fpDes = tri_chassis_turn.leftdown_motor.pos_pid.fpU;
//		tri_chassis_turn.leftdown_motor.velt_pid.CalPID();

//		tri_chassis_turn.rightdown_motor.pos_pid.CalFilterPID();
//		tri_chassis_turn.rightdown_motor.velt_pid.fpDes = tri_chassis_turn.rightdown_motor.pos_pid.fpU;
//		tri_chassis_turn.rightdown_motor.velt_pid.CalPID();
//		break;
//	case POS_LOOP:
//		break;
//	case VELT_LOOP:
//		tri_chassis_turn.up_motor.velt_pid.CalPID();
//		tri_chassis_turn.leftdown_motor.velt_pid.CalPID();
//		tri_chassis_turn.rightdown_motor.velt_pid.CalPID();
//		break;
//	case OPEN_LOOP:
//		if (pre_turn_pid_state != OPEN_LOOP)
//		{
//			tri_chassis_turn.up_motor.velt_pid.fpU = 0;
//			tri_chassis_turn.leftdown_motor.velt_pid.fpU = 0;
//			tri_chassis_turn.rightdown_motor.velt_pid.fpU = 0;
//		}
//		break;
//	}

//	turn_up_current = tri_chassis_turn.up_motor.velt_pid.fpU;
//	turn_leftdown_current = tri_chassis_turn.leftdown_motor.velt_pid.fpU;
//	turn_rightdown_current = tri_chassis_turn.rightdown_motor.velt_pid.fpU;

//	if (tri_chassis_turn.feed_forward_state == WITH_FORWARD)
//	{
//		turn_up_current += tri_chassis_turn.up_motor.feed_forward_current;
//		turn_leftdown_current += tri_chassis_turn.leftdown_motor.feed_forward_current;
//		turn_rightdown_current += tri_chassis_turn.rightdown_motor.feed_forward_current;
//	}

//	/*需要对应修改*/
//	can_send_data(CAN1, 0x200, (int16_t)(run_up_current * k_zoom), turn_up_current,
//				  (int16_t)(run_rightdown_current * k_zoom), turn_rightdown_current);
//	can_send_data(CAN2, 0x200, (int16_t)(run_leftdown_current * k_zoom), turn_leftdown_current,
//				  0, 0);

//	pre_turn_pid_state = tri_chassis_turn.pid_state;
//	pre_run_pid_state = tri_chassis_run.pid_state;
//}

//float dm_motor_kp = 5.0,dm_motor_kd = 0.1,dm_motor_Tff = 0.0;

void ParallelMechanism_control(void)
{
	if (system_state != SYS_RUN)
	{
	  PM_Motor.pm_motor_state = PM_Disable;
	}
	
	switch(PM_Motor.pm_motor_state)
	{
	  case PM_Init:
			PM_Motor.ParallelMechanism_enable_motor();
		break;
		
		case PM_Run:
		  PM_Motor.ParallelMechanism_crl_motor();	
		  PM_Motor.ParallelMechanism_ForwardKinematics_Newton_Iteration();
		break;
		
	  case PM_Disable:
			PM_Motor.ParallelMechanism_set_motor_PID(0,0,0);
			PM_Motor.ParallelMechanism_disable_motor();
		break;
		
	  case PM_Lock:
			
		break;
		
	  case PM_Save_ZeroPoint:
		PM_Motor.ParallelMechanism_save_zeropoint();		
		break;	
		
	  case PM_Error:
			PM_Motor.ParallelMechanism_set_motor_PID(0,0,0);	
		break;		
	}
	
}
