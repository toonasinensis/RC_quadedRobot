#ifndef __NAVIGATION_ALGORITHM_H__
#define __NAVIGATION_ALGORITHM_H__

#include "hit_global_type.h"
#include "math_algorithm.h"
#include "math.h"
#include "chassis.h"   

/********************************************************************************************************
导航相关函数
*********************************************************************************************************/
typedef enum{
	LINE,
	CURVE,
}STATE;
typedef struct
{
	ST_POS pos_ref;
	ST_VECTOR velt_ref;
	ST_VECTOR acc_ref;
	ST_VECTOR curvature;
	fp32 w;
	STATE state;
}ST_Trajectory;


typedef struct{
	float up;
	float leftdown;
	float rightdown;
}tri_chassis_type_t; 


typedef enum
{
	//NAV_CORRENT,
	NAV_OFF,
	NAV_MANUAL,	//手动控制
	NAV_AUTO_PATH, 	//自动路径导航
	NAV_LOCK,	
	NAV_STOP,
	NAV_STOP_ANTI_DIR,
	NAV_CALIBRATION_1,//跑开环，测启动电流与加速度系数
	NAV_CALIBRATION_2,//跑最大加速度，修正加速度系数
	NAV_TEST,
	NAV_SPAN_MANUAL,
	NAV_SPAN_POS_AUTO,
	NAV_SPAN_NEW,
	NAV_INIT,
	NAV_AWAIT,
	NAV_NEW_MANUAL,
	NAV_PATHPLANNING,
	NAV_HIT,
	NAV_STOP_X,
	NAV_MANUAL_PATH,
	NAV_DT35_RELOC,
	NAV_STOP_NEW
}nav_state_e;

typedef struct
{
	cPID x;  //横坐标X（单位：mm）
	cPID y;  //竖坐标Y（单位：mm）
	cPID w;  //航向角Q（单位：0.1度）
}nav_pid_t;
typedef struct
{
	nav_pid_t pos_pid, velt_pid;
	ST_VELT basic_velt;
	ST_VECTOR acceleration, nav_angle;
	uint32_t run_time;
	uint8_t number;
}nav_auto_path_t;




class cNav
{
	public:
	nav_state_e state;
	ST_VECTOR expect_robot_global_velt;
	nav_auto_path_t auto_path;
	
	cNav();
};

#define SET_RESET_Q(flag)                 \
		flag_reset_q = flag;\
			flag_auto_path_q =flag;\

extern void SpeedDistribute_8DegreesOfFreedom(cNav *p_nav);
extern void SpeedDistribute_8DegreesOfFreedom_with_slip_ring(cNav *p_nav,chassis_velt_t *basic_local);
extern void Feedforward_by_curvature(cNav *p_nav,ST_Trajectory* track,chassis_velt_t *basic_local);
extern void SpeedDistribute_6DegreesOfFreedom_with_slip_ring(cNav *p_nav,tri_chassis_velt_t *basic_local);
extern void tri_Feedforward_by_curvature(cNav *p_nav,ST_Trajectory* track,tri_chassis_velt_t *basic_local);
extern void chassis_turn_with_slip_ring(fp32 rightup_des, fp32 rightdown_des, fp32 leftup_des, fp32 leftdown_des);
extern void tri_chassis_turn_with_slip_ring(fp32 up_des, fp32 leftdown_des, fp32 rightdown_des);
extern void cal_chassis_feed_forward(cNav *p_nav);
extern void cal_tri_chassis_feed_forward(cNav *p_nav);
extern void turn_Q(fp32& des,fp32& fb);
extern void PID_4DegreesOfFreedom_Omni_Wheel(cNav *p_nav);

extern fp32 Limit_Angle;
extern  u8 RunMode;
#endif

