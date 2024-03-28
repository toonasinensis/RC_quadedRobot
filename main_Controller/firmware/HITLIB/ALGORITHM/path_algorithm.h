#ifndef __PATH_ALGORITHM_H__
#define __PATH_ALGORITHM_H__

#include "navigation_algorithm.h"
#include "read_remote_ctrl_task.h"

#define SET_NAV_PATH_AUTO(Name)  \
	nav.auto_path.number = Name; \
	nav.state = NAV_AUTO_PATH;   \
	nav.auto_path.run_time = 0;  \
	flag_record = 1;             \
	RunMode = 0;

#define SET_NAV_PATH_MANUAL(Name) \
	nav.auto_path.number = Name;  \
	nav.state = NAV_MANUAL_PATH;  \
	nav.auto_path.run_time = 0;   \
	flag_record = 1;              \
	RunMode = 0;

#define FETCH_LEFT_X_BASE -3125.0f
#define FETCH_LEFT_Y_BASE -1290.0f
#define FETCH_RIGHT_X_BASE 875.53f	 // 890.0f
#define FETCH_RIGHT_Y_BASE -1290.79f //-1280.0f

#define HIT_LAGORI_3_LEFT1_X -1671.13f //-1471.2f
#define HIT_LAGORI_3_LEFT1_Y -1221.88f //-1225.15f
#define HIT_LAGORI_3_LEFT1_Q 5.23f	 // 5.62f

#define HIT_LAGORI_4_LEFT2_X -1671.13f
#define HIT_LAGORI_4_LEFT2_Y -221.88f//-1221.88f 
#define HIT_LAGORI_4_LEFT2_Q 4.93f
#define HIT_LAGORI_4_LEFT1_X -3000.12f//-1911.02f 
#define HIT_LAGORI_4_LEFT1_Y 178.94f//-1119.93f 
#define HIT_LAGORI_4_LEFT1_Q -11.66f//1.01f	 

#define HIT_LAGORI_RIGHT1_X 28.69f//1921.34f	// 41.76f
#define HIT_LAGORI_RIGHT1_Y 141.15f//0.02f 
#define HIT_LAGORI_RIGHT1_Q -24.46f//-0.62f
#define HIT_LAGORI_RIGHT2_X 1199.85f
#define HIT_LAGORI_RIGHT2_Y -1078.38f
#define HIT_LAGORI_RIGHT2_Q -8.98f
#define HIT_R2_X -98.07f //-103.07f
#define HIT_R2_Y 202.68f // 202.43f
//第一球 24.32f 第二球 23.82f
#define HIT_R2_Q 24.47f//24.32f//23.82f
#define HIT_R2_X_2 -98.07f //-103.07f
#define HIT_R2_Y_2 202.68f // 202.43f
#define HIT_R2_Q_2 23.62f//23.52f//23.82f
#define HANDOVER_R_X 1300.0f
#define HANDOVER_R_Y 2880.0f
#define HANDOVER_R_Q 0.0f
#define HANDOVER_B_X -1300.0f
#define HANDOVER_B_Y 2880.0f
#define HANDOVER_B_Q -120.0f

#define LAGORI_R 150.0f

#define DQ 2 * PI / 3

#define CAMERA_OFFSET1 -40.0f
#define CAMERA_OFFSET2 120.0f

#define LEFT_BALL_2_X -1633.03f
#define LEFT_BALL_2_Y -1290.23f
#define LEFT_BALL_2_Q 0.0f

#define LEFT_BALL_1_X -1326.98f
#define LEFT_BALL_1_Y -1290.23f
#define LEFT_BALL_1_Q 0.0f

#define LEFT_BALL_3_X -1933.03f
#define LEFT_BALL_3_Y -1290.23f
#define LEFT_BALL_3_Q 0.0f

#define TOWER_RED_LEFT_X 750.0f
#define TOWER_RED_LEFT_Y 5200.0f//4550.0f
#define TOWER_RED_LEFT_Q -120.0f

#define TOWER_RED_RIGHT_X 2880.0f
#define TOWER_RED_RIGHT_Y 5500.0f 
#define TOWER_RED_RIGHT_Q 60.0f

class POINT
{
public:
	float x_end;
	float y_end;
	float q_end;
	POINT() {}
	POINT(float x, float y, float q)
	{
		x_end = x;
		y_end = y;
		q_end = q;
	}
	void point_set(float x, float y, float q)
	{
		this->x_end = x;
		this->y_end = y;
		this->q_end = q;
	}
};

class VandA
{
public:
	float Vmax;
	float A_up;
	float A_down;
	VandA() {}
	VandA(float vm, float au, float ad)
	{
		Vmax = vm;
		A_up = au;
		A_down = ad;
	}
	void vanda_set(float vm, float au, float ad)
	{
		this->Vmax = vm;
		this->A_up = au;
		this->A_down = ad;
	}
};

extern void chassis_path_choose(cNav *p_nav, ST_Trajectory *track);
bool sin_signal(float &output,float A, float H, float T,float dt,float total_t ,float phi ,float &real_t);
bool ramp_signal(float &output,float des ,float dt,float total_t,uint8_t &flag, float &step);

extern float flag_record; 
extern fp32 fpStartX;	  
extern fp32 fpStartY;	 
extern int times_path_left;
extern int times_path_right;
extern POINT point_end;
extern VandA vanda;

#endif
