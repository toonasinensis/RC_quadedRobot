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
	PM_SERVE_BALL, //����
	PM_MAT_BALL,   //����
	PM_PASS_BALL,  //����
	PM_MANUAL,
	PM_TEST, 

};

typedef struct 
{
	uint8_t path_num;
  float real_t;
}PM_AUTO_PATH;

//ȥĿ��λ�ò����ṹ��
typedef struct 
{
   fp32 alpha ;
   fp32 beta ;
	 fp32 height;
   fp32 T;
	 fp32 dt;
	
}PM_pos_p;

//sin�˶������ṹ��
typedef struct 
{
   fp32 A;        //amplitude
   fp32 T;        //һ�����ڵ�ʱ��
	 fp32 dt;       //һ���������ڵ�ʱ��
   fp32 total_t;  //�ܹ���ʱ��
	 fp32 phi;      //��λ��
   uint8_t type;  //�˶�����
}PM_sin_path_p;

//��������ṹ��
typedef struct 
{
	 //��������
   fp32 alpha1 ; 
   fp32 beta1 ;
	 fp32 height1;
   fp32 T1;
   fp32 td_r1;
   fp32 td_h1;
	
	 //�½�����
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
		
		//�켣��ص�
	void ParallelMechanism_choose_path();
  //����Ŀ��λ�ã����Բ�ֵ��λ�� ����ؽ�
	void ParallelMechanismGotoPosition(PM_pos_p pos_p,cParallelMechanism *p_motor);	
	  //����Ŀ��λ�ã����Բ�ֵ��λ�� ƽ̨
	void ParallelMechanismGotoPosition2(PM_pos_p pos_p,cParallelMechanism *p_motor);	
	//�ӵ�ǰλ�ÿ�ʼ����sin����,
	//sin����  ���룺 ʱ��t  �����y = A * sin(2*pi/T * t + phi)  һ��phi = 0;
  //TΪ���ڣ�total_tΪ��ʱ��, type Ϊѡ��ĳ�����ɶȵ�sin�˶�
	void ParallelMechanismSinPath(PM_sin_path_p sin_p  , cParallelMechanism *p_motor);
	//��ƽ̨TD
	void ParallelMechanism_TD(cParallelMechanism *p_motor);	
	
	
	//�����˶� 
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

