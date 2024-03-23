#ifndef __PRPRLLEL_MECHANISM_H__
#define __PRPRLLEL_MECHANISM_H__

#include "motor_drive.h"
#include "dm_motor.h"
#include "path_algorithm.h"

/*******************并联机构求解常用常数********************/
#define SQRT3  1.73205080757  //根号3



/*******************并联机构参数********************/
#define  TOP_PLATFORM_LENGTH      216.506350946   //上平台边长 125*sqrt(3) 单位mm b
#define  BOTTOM_PLATFORM_LENGTH   216.506350946   //下平台边长 125*sqrt(3) 单位mm a
#define  TOP_PLATFORM_length      125.0           //上平台边   125  单位mm b
#define  BOTTOM_PLATFORM_length   125.0           //下平台边   125  单位mm a

#define  PM_LENGTH_1              160.0           //大臂长 单位mm
#define  PM_LENGTH_2              180.0           //小臂长 单位mm

#define ANGLE_1_MAX      10.0  //运动范围限幅，单位 rad
#define ANGLE_2_MAX      10.0  //运动范围限幅，单位 rad
#define HEIGHT_MAX       340.0  //运动范围限幅，单位 mm
#define HEIGHT_MIN       120.0  //运动范围限幅，单位 mm
#define PM_TCP2PLATFROM  33.0 //TCP到动平台的距离  单位mm

#define INIT_POS_UP      5.5f  //达妙电机初始角度 单位度
#define INIT_POS_RIGHT   5.5f  //达妙电机初始角度 单位度
#define INIT_POS_LEFT    5.5f  //达妙电机初始角度 单位度

/*******************并联机构运动参数********************/
#define START_POSITION_Alpha   0.0f
#define START_POSITION_Beta    0.0f
#define START_POSITION_Height  200.0f

#define Solve_TS  0.002f  //轨迹求解周期


/*******************电机参数********************/
#define VELT_KP 	0.0
#define VELT_KI 	0.0
#define VELT_KD 	0.0
#define VELT_UPMAX 5.0

#define POS_KP 	0.0
#define POS_KI 	0.0
#define POS_KD 	0.0
#define POS_UPMAX 5.0

#define TORQUE_KP 	0.0
#define TORQUE_KI 	0.0
#define TORQUE_KD 	0.0
#define TORQUE_UPMAX 5.0

#define PM_UP_CRL_ID     0x01
#define PM_RIGHT_CRL_ID  0x02
#define PM_LEFT_CRL_ID   0x03

#define PM_UP_FB_ID      0x51
#define PM_RIGHT_FB_ID   0x52
#define PM_LEFT_FB_ID    0x53

enum PM_motor_state
{
    PM_Init,
  	PM_Run,
    PM_Disable,
	  PM_Save_ZeroPoint,
	  PM_Lock,
  	PM_Error,
};


typedef struct 
{
	uint8_t Iteration_k;
	uint8_t max_Iteration_k;
  float accuracy;
}PM_Newton_Iteration;



class cParallelMechanism{
  public:
//	fp32 des_angle1;//alpha
//	fp32 des_angle2;//beta
//	fp32 des_height;
	cPID alpha_pid;
	cPID beta_pid;
	cPID height_pid;
	
	fp32 pre_des_angle1;//alpha
	fp32 pre_des_angle2;//beta
	fp32 pre_des_height;	
	PM_Newton_Iteration pm_FKP;
	PM_motor_state pm_motor_state;
	
	cMotor_dm up_motor;
	cMotor_dm rightdown_motor;
	cMotor_dm leftdown_motor;
  
	cParallelMechanism()
  { 
		pm_motor_state = PM_Disable;
	  up_motor = cMotor_dm();
		rightdown_motor = up_motor;
		leftdown_motor = up_motor;
//  cPID(vKp, vKi, vKd, vUpMax, vEiMax, vUdMax, vEMin, ts);	
		alpha_pid  = cPID(0, 0, 0, 5, 5, 5, 5, 0.001);
		beta_pid   = cPID(0, 0, 0, 5, 5, 5, 5, 0.001);
		height_pid = cPID(0, 0, 0, 5, 5, 5, 5, 0.001);
		
		

		
		pm_FKP.accuracy = 0.000001;
		pm_FKP.Iteration_k = 0;
		pm_FKP.max_Iteration_k = 6;
		
		up_motor.crl_data.ID = PM_UP_CRL_ID;
		up_motor.fb_data.ID = PM_UP_FB_ID;
		rightdown_motor.crl_data.ID = PM_RIGHT_CRL_ID;
		rightdown_motor.fb_data.ID = PM_RIGHT_FB_ID;		
		leftdown_motor.crl_data.ID = PM_LEFT_CRL_ID;
		leftdown_motor.fb_data.ID = PM_LEFT_FB_ID;	
		
	}		
	void ParallelMechanism_para_protection(void);
	void ParallelMechanism_ForwardKinematics(void);
	void ParallelMechanism_ForwardKinematics_Newton_Iteration(void);	
	void ParallelMechanism_InverseKinematics(void);//根据动平台位置，求解电机角度
	
	void ParallelMechanism_update(void);
	void ParallelMechanism_crl_motor(void);
	void ParallelMechanism_enable_motor(void);
	void ParallelMechanism_save_zeropoint(void);
	void ParallelMechanism_disable_motor(void);
	void ParallelMechanism_set_motor_PID(float kp , float kd , float Tff);
		
};


extern cParallelMechanism PM_Motor;



#endif
