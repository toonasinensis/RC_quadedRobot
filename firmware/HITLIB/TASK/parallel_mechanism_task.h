#ifndef __PARALLEL_MECHANISM_TASK_H__
#define __PARALLEL_MECHANISM_TASK_H__


#include "parallel_mechanism.h"
#include "system_monitor.h"
#include "uart_protocol.h"

enum pm_nav_state_e
{
  PM_INIT,
	PM_STOP,
	PM_LOCK,
	PM_SERVE_BALL, //发球
	PM_MAT_BALL,   //垫球
	PM_PASS_BALL,  //传球
	PM_MANUAL,
	PM_TEST, 

};

typedef struct 
{
	uint8_t path_num;
  float real_t;
}PM_AUTO_PATH;

//去目标位置参数结构体
typedef struct 
{
   fp32 alpha ;
   fp32 beta ;
	 fp32 height;
   fp32 T;
	 fp32 dt;
	
}PM_pos_p;

//sin运动参数结构体
typedef struct 
{
   fp32 A;        //amplitude
   fp32 T;        //一个周期的时间
	 fp32 dt;       //一个计算周期的时间
   fp32 total_t;  //总共的时间
	 fp32 phi;      //相位差
   uint8_t type;  //运动类型
}PM_sin_path_p;

//垫球参数结构体
typedef struct 
{
	 //上升参数
   fp32 alpha1 ; 
   fp32 beta1 ;
	 fp32 height1;
   fp32 T1;
   fp32 td_r1;
   fp32 td_h1;
	
	 //下降参数
   fp32 alpha2 ;
   fp32 beta2 ;
	 fp32 height2;
   fp32 T2;
   fp32 td_r2;
   fp32 td_h2;
	
	 fp32 dt;
	 uint8_t type;
	
}PM_mat_p;


class cPM_NAV
{
	public:
	pm_nav_state_e state;
	PM_AUTO_PATH auto_path;
  
	cPM_NAV()
	{
	  state = PM_INIT;
		auto_path.path_num = 0;
		auto_path.real_t = 0;
	}
		
		//轨迹相关的
	void ParallelMechanism_choose_path();
  //给定目标位置，线性插值解位置 电机关节
	void ParallelMechanismGotoPosition(PM_pos_p pos_p,cParallelMechanism *p_motor);	
	  //给定目标位置，线性插值解位置 平台
	void ParallelMechanismGotoPosition2(PM_pos_p pos_p,cParallelMechanism *p_motor);	
	//从当前位置开始，做sin曲线,
	//sin函数  输入： 时间t  输出：y = A * sin(2*pi/T * t + phi)  一般phi = 0;
  //T为周期，total_t为总时间, type 为选择某个自由度的sin运动
	void ParallelMechanismSinPath(PM_sin_path_p sin_p  , cParallelMechanism *p_motor);
	//动平台TD
	void ParallelMechanism_TD(cParallelMechanism *p_motor);	
	
	
	//垫球运动 
	void ParallelMechanismMatBall(PM_mat_p mat_p );
	
	
};



void ParallelMechanism_control_task(void);
extern u8 goto_position_state;	
extern u8 goto_position_state2;	
extern u8 sin_path_state;
extern u8 mat_path_state;
extern cPM_NAV pm_nav;
extern PM_pos_p my_pos_para;
extern PM_pos_p my_pos_para2;
extern PM_sin_path_p my_sin_para ;
extern PM_mat_p change_mat_p;
extern bool vision_alpha_beta_flag;
#endif

