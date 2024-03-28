#ifndef __DM_MOTOR_H__
#define __DM_MOTOR_H__

#include "hit_global_type.h"
#include "bsp_fdcan.h"

//达妙电机参数，必须和调参软件上设置的一样，不然会缩放
#define  DM_P_MIN   -12.5f
#define  DM_P_MAX    12.5f  //单位 rad   换算成度 为5729.58度   
#define  DM_V_MIN   -30.0f  
#define  DM_V_MAX    30.0f   //单位rad/s 换算成r/m  = 381r/m
#define  DM_T_MIN   -20.0f
#define  DM_T_MAX    20.0f

#define  DM_KP_MIN 0.0f
#define  DM_KP_MAX 100.0f
#define  DM_KD_MIN 0.0f
#define  DM_KD_MAX 2.0f

/*************实际电机限位**************/
#define  P_MIN   -4.0f
#define  P_MAX    95.0f  //单位 度 
#define  V_MIN   -30.0f  
#define  V_MAX    30.0f   //单位r/min


enum Motor_State{
dm_Enable,
dm_Disable,
dm_Error	
};

typedef struct
{
    uint16_t ID;
    float T_ff;
    float Velt_des;
    float Pos_des;
    float InvKine_Pos;	
    float pre_Velt_des;
    float pre_Pos_des;	  
    float K_P;
    float K_D;
	  uint16_t send_rate;
	  uint16_t send_cnt;
	  
} dm_motor_send_data_t;

enum dm_error_t
{
  dm_NONE,
	dm_Over_voltage,
	dm_Under_voltage,
	dm_Over_current,
	dm_Mos_error,
	dm_Over_temperature,
	dm_Communication_loss,
	dm_Overload
};

typedef struct
{
    uint16_t ID;
    dm_error_t Error;
    float Pos_fb;	
    float Velt_fb;	
    float pre_Velt_fb;
    float pre_Pos_fb;	
	float T_fb;
	  uint16_t receive_rate;
	  uint16_t receive_cnt;
	
} dm_motor_receive_data_t;




class cMotor_dm{
  public:
	Motor_State motor_state;	
  dm_motor_send_data_t      crl_data;
  dm_motor_receive_data_t   fb_data;
		
	cMotor_dm()
		{
			this->motor_state = dm_Disable;
			
			this->crl_data.ID = 0;
			this->crl_data.T_ff = 0;
      this->crl_data.Velt_des = 0;
      this->crl_data.Pos_des = 0;
      this->crl_data.InvKine_Pos = 0;
			this->crl_data.pre_Pos_des = 0;
			this->crl_data.pre_Velt_des = 0;
      this->crl_data.K_P = 0;
      this->crl_data.K_D = 0;
			
			this->fb_data.ID = 0;
      this->fb_data.Error = dm_NONE;
      this->fb_data.Pos_fb = 0;	
      this->fb_data.Velt_fb = 0;	
			this->fb_data.pre_Pos_fb = 0;
			this->fb_data.pre_Velt_fb = 0;
      this->fb_data.T_fb = 0;
		}
		
		void receive_data(uint8_t data[]);
		void trans_data(_CANMSG *msg);
		void set_motor_PID(float kp , float kd , float Tff);
		void crl_para_protection(void);
		void enable_motor(_CANMSG *msg);
		void save_zero_point(_CANMSG *msg);
		void disable_motor(_CANMSG *msg);
		void SavePositionAsZeroPoint(_CANMSG *msg);
		
};

float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x, float x_min, float x_max, int bits);

#endif

