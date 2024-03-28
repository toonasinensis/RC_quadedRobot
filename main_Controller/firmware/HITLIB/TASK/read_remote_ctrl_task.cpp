#include "read_remote_ctrl_task.h"
#include "navigation_task.h"
#include "parallel_mechanism.h"
#include "parallel_mechanism_task.h"
#include "task_os.h"
#include "vision_task.h"


#define JIN
//#define YY_FF

//#define THREE_BALL
#define FOUR_BALL

uint16_t g_usKeyValue = 0xFFFF;
uint16_t g_usstartKeyValue = 0xFFFF;
int16_t M_SPEED = 700; 
bool flag_auto_fetch = 1;



float q_correct;

keyboard_mode_e keyboard_mode = DEBUG;

waypoint_e waypoint = FETCH_POINT;

u8 flag_save;

bool flag_auto_fetch_all=1;

extern u8 Auto_Key;
extern u8 pre_Auto_Key;
int left_vertical,left_horizontal,right_vertical,right_horizontal;

#ifdef JIN

void Js_Deal(void) 
{

	ReadWlanJsValue(&g_stJsValue);
	left_vertical = (int )(g_stJsValue.usJsLeft>>8);
	left_horizontal = (int )(g_stJsValue.usJsLeft & (0x00ff));
	right_vertical = (int )(g_stJsValue.usJsRight>>8);
	right_horizontal = (int )(g_stJsValue.usJsRight & (0x00ff));

	if(left_vertical>240&&right_vertical>240)
	{		

	}
	else if(left_vertical<10&&right_vertical<10)
	{

	}
	else if(right_vertical<40)
	{
		nav.state = NAV_STOP;
	}
	else
	{
	}
		
}

#endif

#ifdef YY_FF
void Js_Deal(void)
{
}

#endif

uint8_t key_value = 0x00; 
void Key_Deal(void)
{

	ReadWlanKeyValue(&g_usKeyValue); 

	if (PRESS_KEY_11)//手操
	{
	 key_value = 11;
	 nav.state = NAV_MANUAL;
	}
	else if (PRESS_KEY_12)//自动路径
	{
	 key_value = 12;
//	 SET_NAV_PATH_AUTO(22);
//	 nav.state = NAV_AUTO_PATH;
	}
	else if (PRESS_KEY_13)
	{
	 key_value = 13;
	 nav.state = NAV_HIT;

	}
	else if (PRESS_KEY_14)//起来
	{
	 key_value = 14;
	 PM_Motor.ParallelMechanism_set_motor_PID(10.0,0.1,0.0);
	 pm_nav.auto_path.path_num = 3;
	 my_pos_para2.alpha = 0;
	 my_pos_para2.beta = 0;
	 my_pos_para2.height = 160;
	 my_pos_para2.T = 2.0;	
	 goto_position_state2 = 1;
   pm_nav.state = PM_MANUAL;
	}
	else if (PRESS_KEY_15)//回到原点
	{
	 key_value = 15;
	 PM_Motor.ParallelMechanism_set_motor_PID(10.0,0.1,0.0);
	 pm_nav.auto_path.path_num = 3;
	 my_pos_para2.alpha = 0;
	 my_pos_para2.beta = 0;
	 my_pos_para2.height = 140;
	 my_pos_para2.T = 2.0;
	 goto_position_state2 = 1;
	 pm_nav.state = PM_MANUAL;
		
	}
	else if (PRESS_KEY_16)
	{
	 key_value = 16;
	 pm_nav.state = PM_INIT; 

	}
	else if (PRESS_KEY_21)//height
	{
	 key_value = 21;
	 pm_nav.auto_path.path_num = 4;
   change_mat_p.td_r1 = 1000;
	 change_mat_p.height1 = 320.0;
	 change_mat_p.height2 = 130.0;
   change_mat_p.type = 2;
	 pm_nav.state = PM_MAT_BALL;
		
	}
	else if (PRESS_KEY_22)//alpha
	{
	 key_value = 22;
	 pm_nav.auto_path.path_num = 4;
	 change_mat_p.td_r1 = 300;
   change_mat_p.height1 = 300.0;
	 change_mat_p.height2 = 200.0;	
   change_mat_p.type = 2;
	 PM_Motor.ParallelMechanism_set_motor_PID(25.0,0.1,0.0);
	 pm_nav.state = PM_MAT_BALL;
	}
	else if (PRESS_KEY_23)//beta
	{
	 key_value = 23;
   vision_mat_flag = !vision_mat_flag;
	}
	else if (PRESS_KEY_24)//alpha beta
	{
	 key_value = 24;
		hit_vision_flag = !hit_vision_flag;

	}
	else if (PRESS_KEY_25)
	{
	 key_value = 25;
	 mat_path_state	= 1;
	 vision_mat_flag = 1;

	}
	else if (PRESS_KEY_26)
	{
	 key_value = 26;
   pm_nav.state = PM_STOP;

	}
	else if (PRESS_KEY_31)
	{
	 key_value = 31;
	 change_mat_p.td_r1 = 	change_mat_p.td_r1 - 20;
	}
	else if (PRESS_KEY_32)
	{
	 key_value = 32;
	 change_mat_p.td_r1 = 	change_mat_p.td_r1 + 20;

	}
	else if (PRESS_KEY_33)
	{
	 key_value = 33;
	 change_mat_p.height1 = 	change_mat_p.height1 - 20.0;

	}
	else if (PRESS_KEY_34)
	{
	 key_value = 34;
	 change_mat_p.height1 = 	change_mat_p.height1 + 20.0;

	}
	else if (PRESS_KEY_35)
	{
	 key_value = 35;
	 change_mat_p.height2 = 	change_mat_p.height2 - 20.0;

	}
	else if (PRESS_KEY_36)
	{
	 key_value = 36;
	 change_mat_p.height2 = 	change_mat_p.height2 + 20.0;
	}
	else if (PRESS_KEY_40)
	{
	 key_value = 40;
		vision_alpha_beta_flag = !vision_alpha_beta_flag;
		
	 	}
	else if (PRESS_KEY_41)
	{
	 key_value = 41;
	 	}
	else if (PRESS_KEY_42)
	{
	 key_value = 42;

	}
	else if (PRESS_KEY_43)
	{
	 key_value = 43;

	}
  else 
  {
  }	

}

#ifdef THREE_BALL
void Auto_Key_Deal(void)
{
	if (pre_Auto_Key != Auto_Key)
	{
		if (flag_auto_fetch)
		{
			switch (Auto_Key)
			{
			case 1:
				break;
			case 2:
				
				break;
			case 3:
				break;
			default:
				break;
			}
		}
		else
		{
			switch (Auto_Key)
			{
			case 8:
				
				break;
			default:
				break;
			}
		}
		pre_Auto_Key = Auto_Key;
	}
}
#endif

#ifdef FOUR_BALL
void Auto_Key_Deal(void)
{

}
#endif
