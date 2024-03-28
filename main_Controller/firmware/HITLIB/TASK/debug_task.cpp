#include "debug_task.h"

Frame debug_frame = 
{
	{0},
	{0x00, 0x00, 0x80, 0x7f},
};

void debug_updata(void)
{

//	debug_frame.fdata[0] = stRobot.stVelt.fpVx;
//	debug_frame.fdata[1] = stRobot.stVelt.fpVy;
//	debug_frame.fdata[2] = stRobot.stVelt.fpW;
//	
//	debug_frame.fdata[3] = stRobot.stPot.fpPosX;
//	debug_frame.fdata[4] = stRobot.stPot.fpPosY;
//	debug_frame.fdata[5] = stRobot.stPot.fpPosQ;

//	//导航路径相关
//	debug_frame.fdata[6] = nav.expect_robot_global_velt.fpVx;
//	debug_frame.fdata[7] = nav.expect_robot_global_velt.fpVy;
//	debug_frame.fdata[8] = nav.expect_robot_global_velt.fpW;

//	debug_frame.fdata[9] = nav.auto_path.basic_velt.fpVx;
//	debug_frame.fdata[10] = nav.auto_path.basic_velt.fpVy;
//	debug_frame.fdata[11] = nav.auto_path.basic_velt.fpW;

//	debug_frame.fdata[12] = nav.auto_path.pos_pid.x.fpDes;
//	debug_frame.fdata[13] = nav.auto_path.pos_pid.y.fpDes;
//	debug_frame.fdata[14] = nav.auto_path.pos_pid.w.fpDes;

//	debug_frame.fdata[15] = nav.auto_path.run_time;

//	//底盘运动电机相关
//	//速度环
//	debug_frame.fdata[16] = tri_chassis_run.up_motor.velt_pid.fpDes;
//	debug_frame.fdata[17] = tri_chassis_run.leftdown_motor.velt_pid.fpDes;
//	debug_frame.fdata[18] = tri_chassis_run.rightdown_motor.velt_pid.fpDes;

//	debug_frame.fdata[19] = tri_chassis_run.up_motor.velt_pid.fpFB;
//	debug_frame.fdata[20] = tri_chassis_run.leftdown_motor.velt_pid.fpFB;
//	debug_frame.fdata[21] = tri_chassis_run.rightdown_motor.velt_pid.fpFB;

//	//电流
//	debug_frame.fdata[22] = tri_chassis_run.up_motor.velt_pid.fpU;
//	debug_frame.fdata[23] = tri_chassis_run.leftdown_motor.velt_pid.fpU;
//	debug_frame.fdata[24] = tri_chassis_run.rightdown_motor.velt_pid.fpU;

//	debug_frame.fdata[25] = tri_chassis_run.up_motor.feed_forward_current;
//	debug_frame.fdata[26] = tri_chassis_run.leftdown_motor.feed_forward_current;
//	debug_frame.fdata[27] = tri_chassis_run.rightdown_motor.feed_forward_current;

//	debug_frame.fdata[28] = tri_chassis_run.up_motor.real_current;
//	debug_frame.fdata[29] = tri_chassis_run.leftdown_motor.real_current;
//	debug_frame.fdata[30] = tri_chassis_run.rightdown_motor.real_current;

//	//底盘舵轮相关
// 	//位置环
//	debug_frame.fdata[31] = tri_chassis_turn.up_motor.pos_pid.fpDes;
//	debug_frame.fdata[32] = tri_chassis_turn.leftdown_motor.pos_pid.fpDes;
//	debug_frame.fdata[33] = tri_chassis_turn.rightdown_motor.pos_pid.fpDes;

//	debug_frame.fdata[34] = tri_chassis_turn.up_motor.pos_pid.fpFB;
//	debug_frame.fdata[35] = tri_chassis_turn.leftdown_motor.pos_pid.fpFB;
//	debug_frame.fdata[36] = tri_chassis_turn.rightdown_motor.pos_pid.fpFB;

//	//速度环
//	debug_frame.fdata[37] = tri_chassis_turn.up_motor.velt_pid.fpU;
//	debug_frame.fdata[38] = tri_chassis_turn.leftdown_motor.velt_pid.fpU;
//	debug_frame.fdata[39] = tri_chassis_turn.rightdown_motor.velt_pid.fpU;
//	  

//  //转向电机的帧率
//	debug_frame.fdata[40] = rate_monitor.real_rate[0];
//	debug_frame.fdata[41] = rate_monitor.real_rate[1];
//	debug_frame.fdata[42] = rate_monitor.real_rate[2];
//	
//	//行进电机的帧率
//	debug_frame.fdata[43] = rate_monitor.real_rate[3];
//	debug_frame.fdata[44] = rate_monitor.real_rate[4];
//	debug_frame.fdata[45] = rate_monitor.real_rate[5];
//	
//	debug_frame.fdata[46] = dt35_x1;
//	debug_frame.fdata[47] = dt35_x2;
//	debug_frame.fdata[48] = dt35_y1;
//	debug_frame.fdata[49] = dt35_y2;
//	
//	debug_frame.fdata[50] = dt35_now.robot_x;
//	debug_frame.fdata[51] = dt35_now.robot_y;
//	
//	debug_frame.fdata[52] = dt35_now.pro_x1;
//	debug_frame.fdata[53] = dt35_now.pro_x2;
//	debug_frame.fdata[54] = dt35_now.pro_y1;
//	debug_frame.fdata[55] = dt35_now.pro_y2;
}

