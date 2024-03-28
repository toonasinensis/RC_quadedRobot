#include "vision_task.h"
#include "parallel_mechanism_task.h"


ST_LPF LpF_vision_x = {0, 0, 0, 40, 0.002};
ST_LPF LpF_vision_y = {0, 0, 0, 40, 0.002};
ST_LPF LpF_vision_z = {0, 0, 0, 40, 0.002};
//fp32 test_t = 0;
//fp32 test_y = 0;

uint8_t vision_mat_flag = 0;
fp32 vision_mat_limit_max = 500;
fp32 vision_mat_limit_min = 300;
uint8_t vision_falldown_flag = 0;
int vision_cnt = 0;
int vision_period = 1;

fp32 v_max_length = 150;
fp32 vision_angle_max = 3.5;
void vision_process(void)
{
	
	//	test_t = test_t + 0.002;
//	test_y = 10 * sin(2*PI*test_t) + 1 * sin(2*PI*20*test_t);
//	vision_x.in = test_y;
//	LpFilter(&vision_x);
	
	
	//滤一下波
	LpF_vision_x.in = vision_data.x;
	LpF_vision_y.in = vision_data.y;
	LpF_vision_z.in = vision_data.z;
	LpFilter(&LpF_vision_x);
	LpFilter(&LpF_vision_y);
	LpFilter(&LpF_vision_z);
	
	vision_data.x_out = LpF_vision_x.out;
	vision_data.y_out = LpF_vision_y.out;
//	vision_data.z_out = LpF_vision_z.out;
	fp32 L = sqrt(vision_data.x_out*vision_data.x_out + vision_data.y_out*vision_data.y_out);
	if(L > v_max_length )
	{
	  L = v_max_length;
	}
	
	fp32 tan_xy = atan2(vision_data.y_out , vision_data.x_out);
	vision_data.beta_out =  vision_angle_max * L/v_max_length * cosf(tan_xy);
	vision_data.alpha_out =  vision_angle_max	* L/v_max_length * sinf(tan_xy);


	
	//判断在下降且到一定范围
	if( vision_mat_flag)
	{
//	  if((vision_data.pre_z > vision_data.z + 20.0 )&& (vision_data.z < vision_mat_limit_max)&&(vision_data.z > vision_mat_limit_min) )
	  if((vision_data.pre_z > vision_data.z + 20.0 )&& (vision_data.z < vision_mat_limit_max)&&(vision_data.z > vision_mat_limit_min)&&( mat_path_state == 0) )
		{  
    vision_falldown_flag = 1;
		}

		if(vision_falldown_flag == 1)
		{
		vision_cnt++;
		}
		
//		if((vision_falldown_flag ==1)&&(vision_cnt == vision_period))
		if((mat_path_state == 0) && (vision_falldown_flag ==1)&&(vision_cnt == vision_period))
  	{
			 PM_Motor.ParallelMechanism_set_motor_PID(20.0,0.1,0.0);
			
					if(vision_alpha_beta_flag)
					{
					PM_Motor.alpha_pid.fpDes = vision_data.alpha_out;
					PM_Motor.beta_pid.fpDes = vision_data.beta_out;
					}
					else
					{
					PM_Motor.alpha_pid.fpDes = 0;
					PM_Motor.beta_pid.fpDes = 0;					
					}			 			
			
	     mat_path_state	= 1;		
       vision_falldown_flag = 0;	
       vision_cnt	=0;		
		}
	}

//	  if((vision_data.pre_z > vision_data.z + 20.0 )&& (vision_data.z < vision_mat_limit_max)&&(vision_data.z > vision_mat_limit_min)&&( mat_path_state == 0) )
//		{  
//    vision_falldown_flag = 1;
//		}
//		else
//		{
//		vision_falldown_flag = 0;
//		}
//		if(mat_path_state == 0 && vision_falldown_flag ==1)
//  	{
//			 PM_Motor.ParallelMechanism_set_motor_PID(20.0,0.1,0.0);
//	     mat_path_state	= 1;		
//       vision_falldown_flag = 0;			
//		}	

	//更新数据
	vision_data.pre_x = vision_data.x;
	vision_data.pre_y = vision_data.y;
	vision_data.pre_z = vision_data.z;
	
}

