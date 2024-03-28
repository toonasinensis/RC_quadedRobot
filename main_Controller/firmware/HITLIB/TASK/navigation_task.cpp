#include "navigation_task.h"
#include "PM_navigation.h"
#include "uart_protocol.h"
#include "vision_task.h"

#ifdef _PS_
u8 FLAG_INIT_U = 1, FLAG_INIT_LD = 1, FLAG_INIT_RD = 1;
#endif

#ifndef _PS_
u8 FLAG_INIT_U = 0, FLAG_INIT_LD = 0, FLAG_INIT_RD = 0;
#endif

cNav nav;

//float calibration_current = 0;
//float start_calibration_current = 0;



//extern int flag_vision_lagori;
//extern bool flag_auto_fetch;
//int stable_dt35;


//bool with_slip_ring_flag = 1;

bool path_with_pos_pid = 1; // 1
bool hit_vision_flag = 0;
bool flag_reset_q = 0;
bool flag_auto_path_q = 0;
bool use_pos_only = 0;
//bool hit_r2=0;

TD TD_Q_RESET = {0, 0, 0, 0, 90, 0.002, 0.002};

 chassis_velt_t v_circle;
//tri_chassis_velt_t tri_v_circle;
ST_Trajectory path_circle;
//u8 flag_tuoluo;

float des[COM_LENGTH];


int force_out;


void navigation(void)
{
	// int32_t temp_feed_forward_current;
	//类的构造函数有初始化state

	switch (nav.state)
	{
	case NAV_INIT:
#ifndef _PS_
		chassis_turn.pid_state = DOUBLE_LOOP;
		chassis_turn.feed_forward_state = WITHOUT_FORWARD;
		chassis_run.pid_state = VELT_LOOP;
		chassis_run.feed_forward_state = WITHOUT_FORWARD;
#endif

#ifdef _PS_
		//		chassis_turn.pid_state = DOUBLE_LOOP;
		//		chassis_turn.feed_forward_state = WITHOUT_FORWARD;
		//		chassis_run.pid_state = VELT_LOOP;
		//		chassis_run.feed_forward_state = WITHOUT_FORWARD;

		chassis_turn.pid_state = VELT_LOOP;
		chassis_turn.feed_forward_state = WITHOUT_FORWARD;
		chassis_run.pid_state = VELT_LOOP;
		chassis_run.feed_forward_state = WITHOUT_FORWARD;

//		if (system_state == SYS_RUN)
//		{
//			tri_chassis_turn.up_motor.velt_pid.fpDes = 50; //光电门用的
//			tri_chassis_turn.rightdown_motor.velt_pid.fpDes = 50;
//			tri_chassis_turn.leftdown_motor.velt_pid.fpDes = 50;
//		}
//		if (!FLAG_INIT_LD)
//		{
//			tri_chassis_turn.leftdown_motor.velt_pid.fpDes = 0;
//			tri_chassis_turn.leftdown_motor.pos_pid.fpDes = 0;
//		}
//		if (!FLAG_INIT_U)
//		{
//			tri_chassis_turn.up_motor.velt_pid.fpDes = 0;
//			tri_chassis_turn.up_motor.pos_pid.fpDes = 0;
//		}
//		if (!FLAG_INIT_RD)
//		{
//			tri_chassis_turn.rightdown_motor.velt_pid.fpDes = 0;
//			tri_chassis_turn.rightdown_motor.pos_pid.fpDes = 0;
//		}

//		if (!(FLAG_INIT_LD || FLAG_INIT_RD || FLAG_INIT_U))
//		{
//			flag_tuoluo = 1;
//			if (keyboard_mode == PATHPLANNING)
//			{
//				nav.state = NAV_NEW_MANUAL;
//			}
//			else
//			{
//				nav.state = NAV_AWAIT;
//			}
//		}
#endif

		break;

	case NAV_AWAIT:
		chassis_turn.pid_state = DOUBLE_LOOP;
		chassis_turn.feed_forward_state = WITHOUT_FORWARD;
		chassis_run.pid_state = VELT_LOOP;
		chassis_run.feed_forward_state = WITHOUT_FORWARD;
		break;
	case NAV_OFF:
				disable_chassis();
//		disable_tri_chassis();
		break;

	case NAV_STOP: //锁死状态下运动电机改用双环
		stop_chassis();
//		stop_tri_chassis();
		break;

	case NAV_LOCK:
		lock_chassis();
//		lock_tri_chassis();
		break;

	case NAV_MANUAL:
		chassis_turn.pid_state = DOUBLE_LOOP;
		chassis_turn.feed_forward_state = WITHOUT_FORWARD;
		chassis_run.pid_state = VELT_LOOP;
		chassis_run.feed_forward_state = WITHOUT_FORWARD;

		CalSpeedByJoyStick(&g_stJsValue, &nav, 80, M_SPEED, 80, M_SPEED, 80, M_SPEED / 100); //将线速度缩放成0-200，角速度0-100,死区60和60
    PID_4DegreesOfFreedom_Omni_Wheel(&nav);

//		Feedforward_by_curvature(&nav,&path_circle,&v_circle);
//	  SpeedDistribute_8DegreesOfFreedom_with_slip_ring(&nav, &v_circle);

//    tri_Feedforward_by_curvature(&nav,&path_circle,&tri_v_circle)
//		SpeedDistribute_6DegreesOfFreedom_with_slip_ring(&nav, &tri_v_circle);

		break;

	case NAV_NEW_MANUAL: //有fp
		chassis_turn.pid_state = DOUBLE_LOOP;
		chassis_turn.feed_forward_state = WITHOUT_FORWARD;
		chassis_run.pid_state = VELT_LOOP;
		chassis_run.feed_forward_state = WITHOUT_FORWARD;
		if (flag_reset_q)
		{
			nav.auto_path.pos_pid.w.fpFB = stRobot.stPot.fpPosQ / 10.0f;

			if (fabs(nav.auto_path.pos_pid.w.fpFB) < 30)
			{
				nav.auto_path.pos_pid.w.fpDes = 0;
				nav.auto_path.basic_velt.fpW = 0;
				if (fabs(nav.auto_path.pos_pid.w.fpFB) < 1 && fabs(stRobot.stVelt.fpW) < 1)
				{
					flag_reset_q = 0;
				}
			}
			else // TD
			{
				if (flag_auto_path_q)
				{
					TD_Q_RESET.aim = 0;
					TD_Q_RESET.x1 = stRobot.stPot.fpPosQ / 10;
					TD_Q_RESET.x2 = stRobot.stVelt.fpW / 10;
					flag_auto_path_q = 0;
				}
				Clip_TD_Function(&TD_Q_RESET, 90);
				nav.auto_path.pos_pid.w.fpDes = TD_Q_RESET.x1;
				nav.auto_path.basic_velt.fpW = TD_Q_RESET.x2;
			}
			nav.auto_path.pos_pid.w.fpKp = 9.0f;
			nav.auto_path.basic_velt.fpW *= RADIAN;
			nav.auto_path.pos_pid.w.fpDes *= RADIAN;
			nav.auto_path.pos_pid.w.fpFB *= RADIAN;

			nav.auto_path.pos_pid.w.CalPID();

			nav.expect_robot_global_velt.fpVx = 0;
			nav.expect_robot_global_velt.fpVy = 0;
			nav.expect_robot_global_velt.fpW = nav.auto_path.pos_pid.w.fpU + nav.auto_path.basic_velt.fpW;

			ClipFloat(nav.expect_robot_global_velt.fpW, -120 * RADIAN, 120 * RADIAN);

		}
		else
		{
//			CalSpeedByJoyStick(&g_stJsValue, &nav, 80, M_SPEED, 80, M_SPEED, 80, M_SPEED / 100); //将线速度缩放成0-200，角速度0-100,死区60和60
      PID_4DegreesOfFreedom_Omni_Wheel(&nav);
//		  Feedforward_by_curvature(&nav,&path_circle,&v_circle);
//	  SpeedDistribute_8DegreesOfFreedom_with_slip_ring(&nav, &v_circle);
//    tri_Feedforward_by_curvature(&nav,&path_circle,&tri_v_circle)
//			SpeedDistribute_6DegreesOfFreedom_with_slip_ring(&nav, &tri_v_circle);
		}
		break;

	case NAV_AUTO_PATH:

		chassis_turn.pid_state = DOUBLE_LOOP;
		chassis_turn.feed_forward_state = WITHOUT_FORWARD;
		chassis_run.pid_state = VELT_LOOP;
		chassis_run.feed_forward_state = WITH_FORWARD;

		nav.auto_path.pos_pid.x.fpFB = stRobot.stPot.fpPosX;
		nav.auto_path.pos_pid.y.fpFB = stRobot.stPot.fpPosY;
		nav.auto_path.pos_pid.w.fpFB = stRobot.stPot.fpPosQ / 10.0f;

		nav.auto_path.velt_pid.x.fpFB = stRobot.stVelt.fpVx;
		nav.auto_path.velt_pid.y.fpFB = stRobot.stVelt.fpVy;
		nav.auto_path.velt_pid.w.fpFB = stRobot.stVelt.fpW;

		//规划来自路径

		chassis_path_choose(&nav, &path_circle);
		nav.auto_path.pos_pid.x.fpKp = 6.0f;
		nav.auto_path.pos_pid.y.fpKp = 6.0f;
		nav.auto_path.pos_pid.w.fpKp = 7.0f;

		nav.auto_path.basic_velt.fpW *= RADIAN;
		nav.auto_path.pos_pid.w.fpFB *= RADIAN;
		nav.auto_path.pos_pid.w.fpDes *= RADIAN;

		if (path_with_pos_pid)
		{
			nav.auto_path.pos_pid.x.CalPID();
			nav.auto_path.pos_pid.y.CalPID();
			nav.auto_path.pos_pid.w.CalPID();

			nav.expect_robot_global_velt.fpVx = nav.auto_path.pos_pid.x.fpU + nav.auto_path.basic_velt.fpVx;
			nav.expect_robot_global_velt.fpVy = nav.auto_path.pos_pid.y.fpU + nav.auto_path.basic_velt.fpVy;
			nav.expect_robot_global_velt.fpW = nav.auto_path.pos_pid.w.fpU + nav.auto_path.basic_velt.fpW;
		}
		else
		{
			nav.expect_robot_global_velt.fpVx = nav.auto_path.basic_velt.fpVx;
			nav.expect_robot_global_velt.fpVy = nav.auto_path.basic_velt.fpVy;
			nav.expect_robot_global_velt.fpW = nav.auto_path.basic_velt.fpW;
		}

//		Feedforward_by_curvature(&nav, &path_circle, &v_circle);
//		SpeedDistribute_8DegreesOfFreedom_with_slip_ring(&nav, &v_circle);
//    tri_Feedforward_by_curvature(&nav,&path_circle,&tri_v_circle)
//		SpeedDistribute_6DegreesOfFreedom_with_slip_ring(&nav, &tri_v_circle);
		  PID_4DegreesOfFreedom_Omni_Wheel(&nav);
//		  cal_tri_chassis_feed_forward(&nav);

		break;

	case NAV_PATHPLANNING:
		// des1-6是目标位置和目标速度，7是路径规划结束
		chassis_turn.pid_state = DOUBLE_LOOP;
		chassis_turn.feed_forward_state = WITHOUT_FORWARD;
		chassis_run.pid_state = VELT_LOOP;
		chassis_run.feed_forward_state = WITH_FORWARD;

		nav.auto_path.pos_pid.x.fpFB = stRobot.stPot.fpPosX;
		nav.auto_path.pos_pid.y.fpFB = stRobot.stPot.fpPosY;
		nav.auto_path.pos_pid.w.fpFB = stRobot.stPot.fpPosQ / 10.0f;

		nav.auto_path.velt_pid.x.fpFB = stRobot.stVelt.fpVx;
		nav.auto_path.velt_pid.y.fpFB = stRobot.stVelt.fpVy;
		nav.auto_path.velt_pid.w.fpFB = stRobot.stVelt.fpW;

		nav.auto_path.pos_pid.x.fpDes = des[0];
		nav.auto_path.basic_velt.fpVx = des[1];
		nav.auto_path.pos_pid.y.fpDes = des[3];
		nav.auto_path.basic_velt.fpVy = des[4];
		if (use_pos_only)
		{
			nav.auto_path.pos_pid.w.fpDes = 0;
			nav.auto_path.basic_velt.fpW = 0;
		}
		else
		{
			nav.auto_path.pos_pid.w.fpDes = des[5];
			nav.auto_path.basic_velt.fpW = des[6];
		}

		//阶跃太大，切回手动模式
		if (Geometric_mean(nav.auto_path.pos_pid.y.fpDes - stRobot.stPot.fpPosY, nav.auto_path.pos_pid.x.fpFB - stRobot.stPot.fpPosX) > 100)
		{
			nav.state = NAV_NEW_MANUAL;
			break;
		}
		if (!des[7])
		{
			nav.state = NAV_STOP;
		}

		nav.auto_path.pos_pid.x.fpKp = 6.0f;
		nav.auto_path.pos_pid.y.fpKp = 6.0f;
		nav.auto_path.pos_pid.w.fpKp = 9.0f;

		nav.auto_path.basic_velt.fpW *= RADIAN;
		nav.auto_path.pos_pid.w.fpFB *= RADIAN;
		nav.auto_path.pos_pid.w.fpDes *= RADIAN;

		if (path_with_pos_pid)
		{
			nav.auto_path.pos_pid.x.CalPID();
			nav.auto_path.pos_pid.y.CalPID();
			nav.auto_path.pos_pid.w.CalPID();

			nav.expect_robot_global_velt.fpVx = nav.auto_path.pos_pid.x.fpU + nav.auto_path.basic_velt.fpVx;
			nav.expect_robot_global_velt.fpVy = nav.auto_path.pos_pid.y.fpU + nav.auto_path.basic_velt.fpVy;
			nav.expect_robot_global_velt.fpW = nav.auto_path.pos_pid.w.fpU + nav.auto_path.basic_velt.fpW;
		}
		else
		{
			nav.expect_robot_global_velt.fpVx = nav.auto_path.basic_velt.fpVx;
			nav.expect_robot_global_velt.fpVy = nav.auto_path.basic_velt.fpVy;
			nav.expect_robot_global_velt.fpW = nav.auto_path.basic_velt.fpW;
		}

//		SpeedDistribute_6DegreesOfFreedom_with_slip_ring(&nav, &tri_v_circle);
		PID_4DegreesOfFreedom_Omni_Wheel(&nav);
		cal_tri_chassis_feed_forward(&nav);
		break;

	case NAV_HIT:

		chassis_turn.pid_state = DOUBLE_LOOP;
		chassis_turn.feed_forward_state = WITHOUT_FORWARD;
		chassis_run.pid_state = VELT_LOOP;
		chassis_run.feed_forward_state = WITH_FORWARD;

		nav.auto_path.pos_pid.x.fpFB = stRobot.stPot.fpPosX;
		nav.auto_path.pos_pid.y.fpFB = stRobot.stPot.fpPosY;
		nav.auto_path.pos_pid.w.fpFB = stRobot.stPot.fpPosQ / 10.0f;

		nav.auto_path.velt_pid.x.fpFB = stRobot.stVelt.fpVx;
		nav.auto_path.velt_pid.y.fpFB = stRobot.stVelt.fpVy;
		nav.auto_path.velt_pid.w.fpFB = stRobot.stVelt.fpW;

//		nav.auto_path.pos_pid.x.fpDes = point_end.x_end;
//		nav.auto_path.pos_pid.y.fpDes = point_end.y_end;
//		nav.auto_path.pos_pid.w.fpDes = point_end.q_end;
	  if(hit_vision_flag)
		{
		nav.auto_path.pos_pid.x.fpDes = nav.auto_path.pos_pid.x.fpFB + vision_data.x_out;
		nav.auto_path.pos_pid.y.fpDes = nav.auto_path.pos_pid.y.fpFB + vision_data.y_out;
		nav.auto_path.pos_pid.w.fpDes = 0;
		}
		else
		{
    nav.auto_path.pos_pid.x.fpDes = 0;
		nav.auto_path.pos_pid.y.fpDes = 0;
		nav.auto_path.pos_pid.w.fpDes = 0;
		}

//		nav.auto_path.pos_pid.x.fpKp = 6.0f;
//		nav.auto_path.pos_pid.y.fpKp = 6.0f;
//		nav.auto_path.pos_pid.w.fpKp = 6.0f;
		
//		if (fabs(nav.auto_path.pos_pid.x.fpFB - nav.auto_path.pos_pid.x.fpDes) < 10 && fabs(nav.auto_path.pos_pid.y.fpFB - nav.auto_path.pos_pid.y.fpDes) < 10)
//		{
//			nav.auto_path.pos_pid.x.fpKp = 0.0f;
//			nav.auto_path.pos_pid.y.fpKp = 0.0f;
//		}
//		else if (fabs(nav.auto_path.pos_pid.x.fpFB - nav.auto_path.pos_pid.x.fpDes) < 200 && fabs(nav.auto_path.pos_pid.y.fpFB - nav.auto_path.pos_pid.y.fpDes) < 200)
//		{
//			nav.auto_path.pos_pid.x.fpKp = 6.0f;
//			nav.auto_path.pos_pid.y.fpKp = 6.0f;
//		}
//		else
//		{
//			nav.state = NAV_STOP;
//			force_out = 0;
//		}

//		if (fabs(nav.auto_path.pos_pid.w.fpFB - nav.auto_path.pos_pid.w.fpDes) < 5)
//		{
//			nav.auto_path.pos_pid.w.fpKp = 7.0f;
//		}
//		else if (fabs(nav.auto_path.pos_pid.w.fpFB - nav.auto_path.pos_pid.w.fpDes) < 30)
//		{
//			nav.auto_path.pos_pid.w.fpKp = 6.0f;
//		}
//		else
//		{
//			nav.auto_path.pos_pid.w.fpKp = 1.0f;
//		}


		nav.auto_path.basic_velt.fpW *= RADIAN;
		nav.auto_path.pos_pid.w.fpFB *= RADIAN;
		nav.auto_path.pos_pid.w.fpDes *= RADIAN;
		if (path_with_pos_pid)
		{
			nav.auto_path.pos_pid.x.CalPID();
			nav.auto_path.pos_pid.y.CalPID();
			nav.auto_path.pos_pid.w.CalPID();

		  nav.expect_robot_global_velt.fpVx = nav.auto_path.pos_pid.x.fpU;
		  nav.expect_robot_global_velt.fpVy = nav.auto_path.pos_pid.y.fpU;
		  nav.expect_robot_global_velt.fpW = nav.auto_path.pos_pid.w.fpU;
		}
		else
		{
			nav.expect_robot_global_velt.fpVx = nav.auto_path.basic_velt.fpVx;
			nav.expect_robot_global_velt.fpVy = nav.auto_path.basic_velt.fpVy;
			nav.expect_robot_global_velt.fpW = nav.auto_path.basic_velt.fpW;
		}

//		Feedforward_by_curvature(&nav, &path_circle, &v_circle);
//		SpeedDistribute_8DegreesOfFreedom_with_slip_ring(&nav, &v_circle);
//    tri_Feedforward_by_curvature(&nav,&path_circle,&tri_v_circle)
//		SpeedDistribute_6DegreesOfFreedom_with_slip_ring(&nav, &tri_v_circle);
		
		PID_4DegreesOfFreedom_Omni_Wheel(&nav);
//		cal_tri_chassis_feed_forward(&nav);
		

		break;

	case NAV_STOP_X:
		chassis_turn.pid_state = DOUBLE_LOOP;
		chassis_turn.feed_forward_state = WITHOUT_FORWARD;
		chassis_run.pid_state = VELT_LOOP;
		chassis_run.feed_forward_state = WITH_FORWARD;

		nav.auto_path.pos_pid.x.fpFB = stRobot.stPot.fpPosX;
		nav.auto_path.pos_pid.y.fpFB = stRobot.stPot.fpPosY;
		nav.auto_path.pos_pid.w.fpFB = stRobot.stPot.fpPosQ / 10.0f;

		nav.auto_path.velt_pid.x.fpFB = stRobot.stVelt.fpVx;
		nav.auto_path.velt_pid.y.fpFB = stRobot.stVelt.fpVy;
		nav.auto_path.velt_pid.w.fpFB = stRobot.stVelt.fpW;

		nav.auto_path.pos_pid.x.fpDes = point_end.x_end;
		nav.auto_path.pos_pid.y.fpDes = point_end.y_end;
		nav.auto_path.pos_pid.w.fpDes = point_end.q_end;

		if (fabs(nav.auto_path.pos_pid.x.fpFB - nav.auto_path.pos_pid.x.fpDes) < 200 && fabs(nav.auto_path.pos_pid.y.fpFB - nav.auto_path.pos_pid.y.fpDes) < 200)
		{
			nav.auto_path.pos_pid.x.fpKp = 4.5f;
		}
		else
		{
			nav.state = NAV_STOP;
		}

		nav.auto_path.pos_pid.y.fpKp = 0.0f;
		nav.auto_path.pos_pid.w.fpKp = 0.0f;

		nav.auto_path.basic_velt.fpW *= RADIAN;
		nav.auto_path.pos_pid.w.fpFB *= RADIAN;
		nav.auto_path.pos_pid.w.fpDes *= RADIAN;

		nav.auto_path.pos_pid.x.CalPID();
		nav.auto_path.pos_pid.y.CalPID();
		nav.auto_path.pos_pid.w.CalPID();

		nav.expect_robot_global_velt.fpVx = nav.auto_path.pos_pid.x.fpU;
		nav.expect_robot_global_velt.fpVy = nav.auto_path.pos_pid.y.fpU;
		nav.expect_robot_global_velt.fpW = nav.auto_path.pos_pid.w.fpU;

		//		Feedforward_by_curvature(&nav, &path_circle, &v_circle);
		//		SpeedDistribute_8DegreesOfFreedom_with_slip_ring(&nav, &v_circle);

		//    tri_Feedforward_by_curvature(&nav,&path_circle,&tri_v_circle)
//		SpeedDistribute_6DegreesOfFreedom_with_slip_ring(&nav, &tri_v_circle);
		PID_4DegreesOfFreedom_Omni_Wheel(&nav);
		cal_tri_chassis_feed_forward(&nav);

//		stable_x++;

		break;
		
	case NAV_STOP_NEW:
      lock_chassis();
//		lock_tri_chassis_new();
	  
//	  stable_stop++;
	  

	  
		break;

	case NAV_MANUAL_PATH:
		chassis_turn.pid_state = DOUBLE_LOOP;
		chassis_turn.feed_forward_state = WITHOUT_FORWARD;
		chassis_run.pid_state = VELT_LOOP;
		chassis_run.feed_forward_state = WITH_FORWARD;

		//规划来自路径

		chassis_path_choose(&nav, &path_circle);

		nav.expect_robot_global_velt.fpVx = nav.auto_path.basic_velt.fpVx;
		nav.expect_robot_global_velt.fpVy = nav.auto_path.basic_velt.fpVy;
		nav.expect_robot_global_velt.fpW = nav.auto_path.basic_velt.fpW;

//		Feedforward_by_curvature(&nav, &path_circle, &v_circle);
//		SpeedDistribute_8DegreesOfFreedom_with_slip_ring(&nav, &v_circle);
//    tri_Feedforward_by_curvature(&nav,&path_circle,&tri_v_circle)
//		SpeedDistribute_6DegreesOfFreedom_with_slip_ring(&nav, &tri_v_circle);
		PID_4DegreesOfFreedom_Omni_Wheel(&nav);
		cal_tri_chassis_feed_forward(&nav);
		break;

	/*标定各个轮子的直线加速度系数*/
	case NAV_CALIBRATION_1:
//		chassis_turn.pid_state = DOUBLE_LOOP;
//		chassis_turn.feed_forward_state = WITHOUT_FORWARD;
//		chassis_run.pid_state = OPEN_LOOP;
//		chassis_run.feed_forward_state = WITHOUT_FORWARD;

//		/*根据实际要走的方向调整*/
//		chassis_turn.leftdown_motor.pos_pid.fpDes = 0.0f; // x+
//		chassis_turn.rightdown_motor.pos_pid.fpDes = 0.0f;
//		chassis_turn.up_motor.pos_pid.fpDes = 0.0f;

//		chassis_run.up_motor.velt_pid.fpU = calibration_current;
//		chassis_run.rightdown_motor.velt_pid.fpU = calibration_current;
//		chassis_run.leftdown_motor.velt_pid.fpU = calibration_current;

//		if (fabs(stRobot.stPot.fpPosY) > 5000 || fabs(stRobot.stPot.fpPosX) > 5000)
//		{
//			nav.state = NAV_LOCK;
//		}
		break;

	/*标定各个轮子的旋转加速度系数*/
	case NAV_CALIBRATION_2:
		// 		// tri_chassis_turn.pid_state = DOUBLE_LOOP;
		// 		// tri_chassis_turn.feed_forward_state = WITHOUT_FORWARD;
		// 		// tri_chassis_run.pid_state = OPEN_LOOP;
		// 		// tri_chassis_run.feed_forward_state = WITH_FORWARD;

		// 		chassis_turn.pid_state = DOUBLE_LOOP;
		// 		chassis_turn.feed_forward_state = WITHOUT_FORWARD;
		// 		chassis_run.pid_state = OPEN_LOOP;
		// 		chassis_run.feed_forward_state = WITH_FORWARD;

		// 		chassis_turn_with_slip_ring(45, 45, 45, 45);

		// 		/*需要关注镜像*/
		// 		if(chassis_run_state.rightup == FORWARD)
		// 		{
		// 			chassis_run.rightup_motor.velt_pid.fpU = calibration_current;
		// 		}
		// 		else
		// 		{
		// 			chassis_run.rightup_motor.velt_pid.fpU = -calibration_current;
		// 		}

		// 		if(chassis_run_state.rightdown == FORWARD)
		// 		{
		// 			chassis_run.rightdown_motor.velt_pid.fpU = calibration_current;
		// 		}
		// 		else
		// 		{
		// 			chassis_run.rightdown_motor.velt_pid.fpU = -calibration_current;
		// 		}

		// 		if(chassis_run_state.leftup == FORWARD)
		// 		{
		// 			chassis_run.leftup_motor.velt_pid.fpU = calibration_current;
		// 		}
		// 		else
		// 		{
		// 			chassis_run.leftup_motor.velt_pid.fpU = -calibration_current;
		// 		}

		// 		if(chassis_run_state.leftdown == FORWARD)
		// 		{
		// 			chassis_run.leftdown_motor.velt_pid.fpU = -calibration_current;
		// 		}
		// 		else
		// 		{
		// 			chassis_run.leftdown_motor.velt_pid.fpU = calibration_current;
		// 		}

		// //		chassis_run.leftdown_motor.feed_forward_current = START_CURRENT;
		// //		chassis_run.leftup_motor.feed_forward_current = START_CURRENT;
		// //		chassis_run.rightdown_motor.feed_forward_current = START_CURRENT;
		// //		chassis_run.rightup_motor.feed_forward_current = START_CURRENT;

		break;
	
	case NAV_DT35_RELOC:
//		stable_dt35++;
//		tri_chassis_turn.pid_state = DOUBLE_LOOP;
//		tri_chassis_turn.feed_forward_state = WITHOUT_FORWARD;
//		tri_chassis_run.pid_state = VELT_LOOP;
//		tri_chassis_run.feed_forward_state = WITH_FORWARD;
//	
//	  nav.expect_robot_global_velt.fpVx = 0;
//	  nav.expect_robot_global_velt.fpVy = -400;
//    nav.expect_robot_global_velt.fpW = 0;
//	
//	  if(stable_dt35>=800)
//		{
//			stRobot.stPot.fpPosQ1=-4.1;
//			stRobot.stPot.fpPosQ=-4.1;
//			DT35_relocation_new(&stRobot, &stFollowerWheel, &dt35_save, &dt35_now);
//		}
//		if (stable_dt35>=1200)
//		{
//			nav.state = NAV_STOP_X;
//			stable_dt35=0;
//			reloc_x_y_q=0;
//		}
//		
//		SpeedDistribute_6DegreesOfFreedom_with_slip_ring(&nav, &tri_v_circle);
		break;
	case NAV_TEST:
//		tri_chassis_turn.pid_state = DOUBLE_LOOP;
//		tri_chassis_turn.feed_forward_state = WITHOUT_FORWARD;
//		tri_chassis_run.pid_state = OPEN_LOOP;
//		tri_chassis_run.feed_forward_state = WITHOUT_FORWARD;

//		tri_chassis_run.leftdown_motor.velt_pid.fpU = start_calibration_current;
//		tri_chassis_run.rightdown_motor.velt_pid.fpU = start_calibration_current;
//		tri_chassis_run.up_motor.velt_pid.fpU = start_calibration_current;
		chassis_turn.pid_state = DOUBLE_LOOP;
		chassis_turn.feed_forward_state = WITHOUT_FORWARD;
		chassis_run.pid_state = OPEN_LOOP;
		chassis_run.feed_forward_state = WITHOUT_FORWARD;

//		CalSpeedByJoyStick(&g_stJsValue, &nav, 80, M_SPEED, 80, M_SPEED, 80, M_SPEED / 100); //将线速度缩放成0-200，角速度0-100,死区60和60
      PID_4DegreesOfFreedom_Omni_Wheel(&nav);

		//所有状态在debug中进行修改
		break;

	default:
		break;
	}
}


