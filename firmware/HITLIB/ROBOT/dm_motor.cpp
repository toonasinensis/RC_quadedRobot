#include "dm_motor.h"


void cMotor_dm::receive_data(uint8_t data[])
{
	   float temp_angle, temp_velt;
	   int16_t p_int, v_int, t_int;
	   float  position, velocity, torque;

    	p_int=(data[1]<<8)| data[2];
      v_int=(data[3]<<4)|(data[4]>>4);
      t_int=((data[4]&0xF)<<8)|data[5];
	
      position = uint_to_float(p_int, 0, 2*DM_P_MAX, 16);
      velocity = uint_to_float(v_int, DM_V_MIN, DM_V_MAX, 12); 
      torque = uint_to_float(t_int, DM_T_MIN, DM_T_MAX, 12);  
	
			if(position<0)
			{  position = position + DM_P_MAX;  }
      else
			{  position = position - DM_P_MAX;	 }				
			temp_angle = (position * DEG) ; //单位转换为 度  输出轴
			temp_velt = velocity / (2 * PI) * 60; //单位从rad/s 转换为 转每分 r/m		
			
			this->fb_data.Pos_fb = temp_angle;
      this->fb_data.Velt_fb = temp_velt;
			this->fb_data.T_fb = torque;
}

void cMotor_dm::trans_data(_CANMSG *msg)
{
    uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	  float position, velocity;
	  //输入限幅
	  this->crl_para_protection();
	  
	  position = this->crl_data.Pos_des * RADIAN; //单位转换为rad
	  velocity = this->crl_data.Velt_des * PI / 30;
	
    pos_tmp = float_to_uint(position,  DM_P_MIN,  DM_P_MAX,  16);
    vel_tmp = float_to_uint(velocity, DM_V_MIN,  DM_V_MAX,  12);
    kp_tmp  = float_to_uint(this->crl_data.K_P,      DM_KP_MIN, DM_KP_MAX, 12);
    kd_tmp  = float_to_uint(this->crl_data.K_D,      DM_KD_MIN, DM_KD_MAX, 12);
    tor_tmp = float_to_uint(this->crl_data.T_ff,    DM_T_MIN,  DM_T_MAX,  12);	

	
    msg->id = this->crl_data.ID;
    msg->buffer[0] = (pos_tmp >> 8);
    msg->buffer[1] = pos_tmp;
    msg->buffer[2] = (vel_tmp >> 4);
    msg->buffer[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
    msg->buffer[4] = kp_tmp;
    msg->buffer[5] = (kd_tmp >> 4);
    msg->buffer[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
    msg->buffer[7] = tor_tmp;
}
void cMotor_dm::crl_para_protection(void)
{
  if(this->crl_data.Pos_des > P_MAX) 
	{
	  this->crl_data.Pos_des = P_MAX;
	}
  if(this->crl_data.Pos_des < P_MIN) 
	{
	  this->crl_data.Pos_des = P_MIN;
	}
	
	if(this->crl_data.Velt_des > V_MAX) 
	{
	  this->crl_data.Velt_des = V_MAX;
	}
  if(this->crl_data.Velt_des < V_MIN) 
	{
	  this->crl_data.Velt_des = V_MIN;
	}
	
}
void cMotor_dm::set_motor_PID(float kp , float kd , float Tff)
{
	this->crl_data.K_P = kp;
  this->crl_data.K_D = kd;
	this->crl_data.T_ff = Tff;
}
	

void cMotor_dm::enable_motor(_CANMSG *msg)
{
		msg->id = this->crl_data.ID;
    msg->buffer[0] = 0xFF;
    msg->buffer[1] = 0xFF;
    msg->buffer[2] = 0xFF;
    msg->buffer[3] = 0xFF;
    msg->buffer[4] = 0xFF;
    msg->buffer[5] = 0xFF;
    msg->buffer[6] = 0xFF;
    msg->buffer[7] = 0xFC;
}

void cMotor_dm::save_zero_point(_CANMSG *msg)
{
		msg->id = this->crl_data.ID;
    msg->buffer[0] = 0xFF;
    msg->buffer[1] = 0xFF;
    msg->buffer[2] = 0xFF;
    msg->buffer[3] = 0xFF;
    msg->buffer[4] = 0xFF;
    msg->buffer[5] = 0xFF;
    msg->buffer[6] = 0xFF;
    msg->buffer[7] = 0xFE;
}
void cMotor_dm::disable_motor(_CANMSG *msg)
{
		msg->id = this->crl_data.ID;
    msg->buffer[0] = 0xFF;
    msg->buffer[1] = 0xFF;
    msg->buffer[2] = 0xFF;
    msg->buffer[3] = 0xFF;
    msg->buffer[4] = 0xFF;
    msg->buffer[5] = 0xFF;
    msg->buffer[6] = 0xFF;
    msg->buffer[7] = 0xFD;
}

void cMotor_dm::SavePositionAsZeroPoint(_CANMSG *msg)
{
		msg->id = this->crl_data.ID;
    msg->buffer[0] = 0xFF;
    msg->buffer[1] = 0xFF;
    msg->buffer[2] = 0xFF;
    msg->buffer[3] = 0xFF;
    msg->buffer[4] = 0xFF;
    msg->buffer[5] = 0xFF;
    msg->buffer[6] = 0xFF;
    msg->buffer[7] = 0xFE;
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
 // converts unsigned int to float, given range and number of bits ///
 float span = x_max - x_min;
 float offset = x_min;
 return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
 }

int float_to_uint(float x, float x_min, float x_max, int bits)
{
   // Converts a float to an unsigned int, given range and number of bits
   float span = x_max - x_min;
   float offset = x_min;
   return (int) ((x-offset)*((float)((1<<bits)-1))/span);
 }

