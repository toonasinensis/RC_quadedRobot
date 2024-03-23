#include "parallel_mechanism_task.h"

cPM_NAV pm_nav;

u8 goto_position_state = 0;	
PM_pos_p my_pos_para = {0,0,220,2.0,0.002};
//平台线性插值参数
u8 goto_position_state2 = 0;	
PM_pos_p my_pos_para2 = {0,0,220,2.0,0.002};

u8 sin_path_state = 0;	
PM_sin_path_p my_sin_para = {80.0,2.0,0.002,6.0,0,0};

u8 mat_path_state = 0;	
fp32 TD_turn_R = 350.0f;
fp32 TD_turn_H = 0.01f;
fp32 TD_turn_T = 0.01f;
//alpha1  beta1  height1  T1  td_r1  td_h1;
PM_mat_p my_mat_p = {0,0,300,2.0,TD_turn_R,TD_turn_H,   0,0,200,2.0,TD_turn_R,TD_turn_H,  0.002 , 1 };
PM_mat_p change_mat_p = {0,0,300,2.0,300,TD_turn_H,   0,0,200,2.0,220,TD_turn_H,  0.002 , 1 };

bool vision_alpha_beta_flag = 0;

void ParallelMechanism_control_task(void)
{
	if (system_state == SYS_RUN)
	{
	
	 switch(pm_nav.state)
	 {
		 case PM_INIT:
		      PM_Motor.pm_motor_state = PM_Init;
		 break;
		 
		 case PM_MANUAL:
		 		  PM_Motor.pm_motor_state = PM_Run;
		 break;
		 
		 case PM_STOP:
		      PM_Motor.pm_motor_state = PM_Disable;
		 break;
		 
		 case PM_LOCK:
		      PM_Motor.pm_motor_state = PM_Lock;
		 break;
		 
		 case PM_SERVE_BALL://发球
		 		  PM_Motor.pm_motor_state = PM_Run;
		 
		 break;
		 
		 case PM_MAT_BALL://垫球
		 		  PM_Motor.pm_motor_state = PM_Run;
		      if(mat_path_state == 0)
					{
						my_mat_p = change_mat_p;
					}

//          PM_Motor.beta_pid.fpDes = -0.4;					
		      pm_nav.ParallelMechanismMatBall(my_mat_p);
								
		 
		 break;
		 
		 case PM_PASS_BALL://传球
		 		  PM_Motor.pm_motor_state = PM_Run;
		 
		 break;
		 
		 case PM_TEST:
		 
		 break;		 
		 
		 default:
		 break;
	 
	 }		 
		
		
	 pm_nav.ParallelMechanism_choose_path();
	 PM_Motor.ParallelMechanism_update();
	 
	 
	}
}

void cPM_NAV::ParallelMechanism_choose_path(void)
{
  	switch (this->auto_path.path_num)
	  {
	  case 0:
		  // none
		
		break;
	  case 1://start position

		this->ParallelMechanismGotoPosition(my_pos_para,&PM_Motor);
		break;
	  case 2://sin path
		this->ParallelMechanismSinPath(my_sin_para,&PM_Motor);	
		break;
	  case 3://上平台插值
		this->ParallelMechanismGotoPosition2(my_pos_para2,&PM_Motor);		
	  break;
	  case 4://上平台TD
		this->ParallelMechanism_TD(&PM_Motor);		
	  break;
		
	  default:
		break;
    }	
}




fp32 UP_ANGLE = 0;
fp32 RIGHT_ANGLE = 0;
fp32 LEFT_ANGLE = 0;
uint8_t pm_motor_flag[3] = {0};
fp32 pm_motor_step[3] = {0};
bool Position_finish_flag[3] = {0};

void cPM_NAV::ParallelMechanismGotoPosition(PM_pos_p pos_p,cParallelMechanism *p_motor)
{
	
    switch (goto_position_state)
    {
    case 1: //求解电机角度
    {
		 p_motor->alpha_pid.fpDes = pos_p.alpha;
		 p_motor->beta_pid.fpDes = pos_p.beta;
		 p_motor->height_pid.fpDes = pos_p.height;
     p_motor->ParallelMechanism_InverseKinematics();
		 UP_ANGLE = p_motor->up_motor.crl_data.Pos_des;
		 RIGHT_ANGLE = p_motor->rightdown_motor.crl_data.Pos_des;
		 LEFT_ANGLE = p_motor->leftdown_motor.crl_data.Pos_des;
			
		 p_motor->up_motor.crl_data.Pos_des        = p_motor->up_motor.crl_data.pre_Pos_des;
		 p_motor->rightdown_motor.crl_data.Pos_des = p_motor->rightdown_motor.crl_data.pre_Pos_des;
		 p_motor->leftdown_motor.crl_data.Pos_des  = p_motor->leftdown_motor.crl_data.pre_Pos_des;
			
		 pm_motor_flag[0] = 1;	
		 pm_motor_flag[1] = 1;	
		 pm_motor_flag[2] = 1;	

		 goto_position_state = 2;	
    }
    break;

    case 2: //插补法获得0—T时刻电机角度值（区别于按时间规划的步态）
    {
		 Position_finish_flag[0] = 
			  ramp_signal(p_motor->up_motor.crl_data.Pos_des,UP_ANGLE,pos_p.dt,
			                pos_p.T,pm_motor_flag[0],pm_motor_step[0]);
		 Position_finish_flag[1] =
			  ramp_signal(p_motor->rightdown_motor.crl_data.Pos_des,RIGHT_ANGLE,pos_p.dt,
			                pos_p.T,pm_motor_flag[1],pm_motor_step[1]);
		 Position_finish_flag[2] =
			  ramp_signal(p_motor->leftdown_motor.crl_data.Pos_des,LEFT_ANGLE,pos_p.dt,
		                  pos_p.T,pm_motor_flag[2],pm_motor_step[2]);
		
			if(Position_finish_flag[0]&&Position_finish_flag[1]&&Position_finish_flag[2])
			{
       goto_position_state = 10;
			}
    }
    break;

    case 10: {
        goto_position_state = 0;
     }
       break;

     default:
     break;
    }	

}

fp32 ALPHA_D,BETA_D,HEIGHT_D;
// 平台线性插值
void cPM_NAV::ParallelMechanismGotoPosition2(PM_pos_p pos_p,cParallelMechanism *p_motor)
{
	
    switch (goto_position_state2)
    {
    case 1: //求解
    {
			
		 ALPHA_D  = pos_p.alpha;
		 BETA_D   = pos_p.beta;
		 HEIGHT_D = pos_p.height;
			
		 pm_motor_flag[0] = 1;	
		 pm_motor_flag[1] = 1;	
		 pm_motor_flag[2] = 1;	

		 goto_position_state2 = 2;	
    }
    break;

    case 2: //插补法获得0—T时刻电机角度值（区别于按时间规划的步态）
    {
		 Position_finish_flag[0] = 
			  ramp_signal(p_motor->alpha_pid.fpDes,ALPHA_D,pos_p.dt,pos_p.T,pm_motor_flag[0],pm_motor_step[0]);
		 Position_finish_flag[1] =
			  ramp_signal(p_motor->beta_pid.fpDes,BETA_D,pos_p.dt,pos_p.T,pm_motor_flag[1],pm_motor_step[1]);
		 Position_finish_flag[2] =
			  ramp_signal(p_motor->height_pid.fpDes,HEIGHT_D,pos_p.dt,pos_p.T,pm_motor_flag[2],pm_motor_step[2]);
      
			p_motor->ParallelMechanism_InverseKinematics();		
			
			if(Position_finish_flag[0]&&Position_finish_flag[1]&&Position_finish_flag[2])
			{
       goto_position_state2 = 10;
			}
    }
    break;

    case 10: {
        goto_position_state2 = 0;
     }
       break;

     default:
     break;
    }	

}




TD alpha_td = {0,0,0,0,TD_turn_R,TD_turn_H,TD_turn_T,0};
TD beta_td =  {0,0,0,0,TD_turn_R,TD_turn_H,TD_turn_T,0};
TD height_td = {0,0,0,0,TD_turn_R,TD_turn_H,TD_turn_T,0};

u8 TD_path_state = 0;	
void cPM_NAV::ParallelMechanism_TD(cParallelMechanism *p_motor)
{
	
 switch (TD_path_state)
    {
    case 1://设置参数 
    {
			alpha_td.aim = p_motor->alpha_pid.fpDes;
			beta_td.aim = p_motor->beta_pid.fpDes;
			height_td.aim = p_motor->height_pid.fpDes;
			
			alpha_td.x1 = p_motor->pre_des_angle1;
			beta_td.x1 = p_motor->pre_des_angle2;
			height_td.x1 = p_motor->pre_des_height;
			
			TD_path_state = 2;
    }
    break;

    case 2:
    {
//        TD_Function(&alpha_td);  
//        TD_Function(&beta_td);  
        TD_Function(&height_td);
//			  p_motor->alpha_pid.fpDes = alpha_td.x1;
//			  p_motor->beta_pid.fpDes = beta_td.x1;
			  p_motor->height_pid.fpDes = height_td.x1;
			  p_motor->ParallelMechanism_InverseKinematics();		
			
			  if( fabs(alpha_td.x) < 0.001&& fabs(beta_td.x) < 0.001&&fabs(height_td.x) < 0.001)
				{
				 TD_path_state = 10;			
				}
    }
    break;

    case 10: {
        TD_path_state = 0;
     }
       break;

     default:
     break;
    }	
	
	

}



bool sin_path_finish_flag[3] = {0};
fp32 SIN_PATH_HIGHT = 0;
void cPM_NAV::ParallelMechanismSinPath(PM_sin_path_p sin_p  , cParallelMechanism *p_motor)
{
  if(sin_p.type == 1)//Height 做sin运动
	{
	 switch(sin_path_state)
	 {
		case 1:
			PM_Motor.alpha_pid.fpDes = 0;
		  PM_Motor.beta_pid.fpDes = 0;
		  this->auto_path.real_t = 0.0f;
		  SIN_PATH_HIGHT = PM_Motor.pre_des_height;
			
		  sin_path_state = 2;
		break;
		case 2:
		  sin_path_finish_flag[0] = sin_signal(PM_Motor.height_pid.fpDes ,sin_p.A ,
		       SIN_PATH_HIGHT, sin_p.T , sin_p.dt , sin_p.total_t , sin_p.phi ,this->auto_path.real_t);
      PM_Motor.ParallelMechanism_InverseKinematics();
		  if(sin_path_finish_flag[0] == 1)
			 {
		   sin_path_state = 10;
			 }
		break;
		case 10:

  		sin_path_state = 0;
		break;		
    default:
    break;		
	 }		
	}
	else if(sin_p.type == 2)//alpha 做sin运动
	{
	 switch(sin_path_state)
	 {
		case 1:
		  PM_Motor.beta_pid.fpDes = 0;
		  this->auto_path.real_t = 0.0f;
			
		  sin_path_state = 2;
		break;
		case 2:
		  sin_path_finish_flag[1] = sin_signal(PM_Motor.alpha_pid.fpDes , sin_p.A/10 ,0, 
		        sin_p.T , sin_p.dt , sin_p.total_t , sin_p.phi ,this->auto_path.real_t);
      PM_Motor.ParallelMechanism_InverseKinematics();
		  if(sin_path_finish_flag[1] == 1)
			 {
		   sin_path_state = 10;
			 }
		break;
		case 10:

  		sin_path_state = 0;
		break;		
    default:
    break;		
	 }
	}
	else if(sin_p.type == 3)//beta 做sin运动
	{
	 switch(sin_path_state)
	 {
		case 1:
		  PM_Motor.alpha_pid.fpDes = 0;
		  this->auto_path.real_t = 0.0f;
			
		  sin_path_state = 2;
		break;
		case 2:
		  sin_path_finish_flag[2] = sin_signal(PM_Motor.beta_pid.fpDes , sin_p.A/10 ,0, 
		      sin_p.T , sin_p.dt , sin_p.total_t , sin_p.phi ,this->auto_path.real_t);
      PM_Motor.ParallelMechanism_InverseKinematics();
		  if(sin_path_finish_flag[2] == 1)
			 {
		   sin_path_state = 10;
			 }
		break;
		case 10:

  		sin_path_state = 0;
		break;		
    default:
    break;		
	 }
	}
	else if(sin_p.type == 4)//alpha 和 beta 做sin运动
	{
	 switch(sin_path_state)
	 {
		case 1:
		  this->auto_path.real_t = 0.0f;
			
		  sin_path_state = 2;
		break;
		case 2:
		  sin_path_finish_flag[1] = sin_signal(PM_Motor.alpha_pid.fpDes , sin_p.A/10 ,0, 
		      sin_p.T , sin_p.dt , sin_p.total_t , sin_p.phi ,this->auto_path.real_t);
		  sin_path_finish_flag[2] = sin_signal(PM_Motor.beta_pid.fpDes , sin_p.A/10 ,0, 
		      sin_p.T , sin_p.dt , sin_p.total_t , sin_p.phi ,this->auto_path.real_t);
      PM_Motor.ParallelMechanism_InverseKinematics();
		  if(sin_path_finish_flag[1] == 1 && sin_path_finish_flag[2] == 1)
			 {
		   sin_path_state = 10;
			 }
		break;
		case 10:

  		sin_path_state = 0;
		break;		
    default:
    break;		
	 }
	}
	else if(sin_p.type == 5)//height alpha beta 做sin运动
	{
	 switch(sin_path_state)
	 {
		case 1:
		  this->auto_path.real_t = 0.0f;
		  SIN_PATH_HIGHT = PM_Motor.pre_des_height;
		
		  sin_path_state = 2;
		break;
		case 2:
		  sin_path_finish_flag[0] = sin_signal(PM_Motor.height_pid.fpDes , sin_p.A ,SIN_PATH_HIGHT , 
		     sin_p.T , sin_p.dt , sin_p.total_t , sin_p.phi ,this->auto_path.real_t);
		  sin_path_finish_flag[1] = sin_signal(PM_Motor.alpha_pid.fpDes , sin_p.A /10 ,0 , 
		     sin_p.T , sin_p.dt , sin_p.total_t , sin_p.phi ,this->auto_path.real_t);
		  sin_path_finish_flag[2] = sin_signal(PM_Motor.beta_pid.fpDes , sin_p.A /10 ,0 , 
		     sin_p.T , sin_p.dt , sin_p.total_t , sin_p.phi ,this->auto_path.real_t);

  		PM_Motor.ParallelMechanism_InverseKinematics();
		  if(sin_path_finish_flag[0] == 1 && sin_path_finish_flag[1] == 1 && sin_path_finish_flag[2] == 1 )
			 {
		   sin_path_state = 10;
			 }
		break;
		case 10:

  		sin_path_state = 0;
		break;		
    default:
    break;		
	 }
	}
	else
	{}

}


void cPM_NAV::ParallelMechanismMatBall(PM_mat_p mat_p )
{
	  if(mat_p.type == 1)//动平台线性插值
		{
	  
    switch (mat_path_state)
    {
    case 1: //上升阶段
    {
     this->auto_path.path_num = 3;//选择goto_position
//		 my_pos_para2.alpha  = mat_p.alpha1;
//		 my_pos_para2.beta   = mat_p.beta1;
		 my_pos_para2.height = mat_p.height1;
		 my_pos_para2.T      = mat_p.T1;
		 my_pos_para2.dt     = mat_p.dt;

			
		 goto_position_state2 = 1;	
		 mat_path_state = 2;	
		 
    }
    break;
    case 2: //上升阶段完成
    {
		 if(goto_position_state2 == 0)//上升阶段完成了
     {
		 mat_path_state = 3;
		 }		 
    }
    break;
		
    case 3: //下升阶段
    {
     this->auto_path.path_num = 3;//选择goto_position
//		 my_pos_para2.alpha  = mat_p.alpha2;
//		 my_pos_para2.beta   = mat_p.beta2;
		 my_pos_para2.height = mat_p.height2;
		 my_pos_para2.T      = mat_p.T2;
		 my_pos_para2.dt     = mat_p.dt;

			
		 goto_position_state2 = 1;	
		 mat_path_state = 4;

    }
    break;
    case 4: 
		{
      if(goto_position_state2 == 0)//下升阶段完成了
     {
			mat_path_state = 10;
		 }
    }
    break;
		 
    case 10: {
        mat_path_state = 0;
     }
       break;

     default:
     break;
    }			
		}
		
		
		
    else if(mat_p.type == 2)//动平台位置TD轨迹
		{
    switch (mat_path_state)
    {
    case 1: //上升阶段
    {
     this->auto_path.path_num = 4;//选择td
			alpha_td.r = mat_p.td_r1;
			alpha_td.h = mat_p.td_h1;
			beta_td.r = mat_p.td_r1;
			beta_td.h = mat_p.td_h1;
			height_td.r = mat_p.td_r1;
			height_td.h = mat_p.td_h1;
			
//			PM_Motor.alpha_pid.fpDes = mat_p.alpha1;
//			PM_Motor.beta_pid.fpDes = mat_p.beta1;
			PM_Motor.height_pid.fpDes = mat_p.height1;
			
		  TD_path_state = 1;	
		  mat_path_state = 2;	
		 
    }
    break;
    case 2: //上升阶段完成
    {
		 if(TD_path_state == 0)//上升阶段完成了
     {
		 mat_path_state = 3;
		 }		 
    }
    break;
		
    case 3: //下升阶段
    {
     this->auto_path.path_num = 4;//选择td
			alpha_td.r = mat_p.td_r2;
			alpha_td.h = mat_p.td_h2;
			beta_td.r = mat_p.td_r2;
			beta_td.h = mat_p.td_h2;
			height_td.r = mat_p.td_r2;
			height_td.h = mat_p.td_h2;
			
//			PM_Motor.alpha_pid.fpDes = mat_p.alpha2;
//			PM_Motor.beta_pid.fpDes = mat_p.beta2;
			PM_Motor.height_pid.fpDes = mat_p.height2;

			
		 TD_path_state = 1;	
		 mat_path_state = 4;

    }
    break;
    case 4: 
		{
      if(TD_path_state == 0)//下升阶段完成了
     {
			mat_path_state = 10;
		 }
    }
    break;
		 
    case 10: {
        mat_path_state = 0;
     }
       break;

     default:
     break;
    }				
		
		}
		else
		{}
    		

}




