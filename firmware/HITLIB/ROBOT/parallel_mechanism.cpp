#include "parallel_mechanism.h"
#include "path_algorithm.h"

cParallelMechanism PM_Motor ;



//并联机构正运动学
void cParallelMechanism::ParallelMechanism_ForwardKinematics(void)
{
	
}

float pm_fkp_a = BOTTOM_PLATFORM_LENGTH * 0.001;//单位转换为m
float pm_fkp_b = TOP_PLATFORM_LENGTH * 0.001;//单位转换为m
float pm_fkp_L1 = PM_LENGTH_1 * 0.001;  //单位转换为m
float pm_fkp_L2 = PM_LENGTH_2 * 0.001;   //单位转换为m
float O2B1 = 1.5*pm_fkp_b;
float O2B2 = 0.5*1.73205080757*pm_fkp_b;
float A1E = 1.5*pm_fkp_a;
float A2E = 0.5*1.73205080757*pm_fkp_a;
float A1D1 = pm_fkp_L1;
float B1D1 = pm_fkp_L2;

//不变的参数
float fkp_a4 = 2*O2B1;
float fkp_b4 = 2*O2B2;
float fkp_c4 = -2*O2B2;//O2B2 = O2B3
float INIT_POS_test = 5.5;
//并联机构正运动学 牛顿迭代法
// 一些参数不变，就不在函数里计算了
void cParallelMechanism::ParallelMechanism_ForwardKinematics_Newton_Iteration(void)
{
  //初始化一些参数
	uint8_t  flag =1; //迭代终止标志位
  fp32 alpha,beta,H,temp,theta1,theta2,theta3;
  this->pm_FKP.Iteration_k = 0;
	this->ParallelMechanism_para_protection();
	alpha = this->alpha_pid.fpDes * RADIAN;//转化为弧度
  beta =  this->beta_pid.fpDes * RADIAN;//转化为弧度
//  temp = cosf(beta)*sinf(alpha)* TOP_PLATFORM_length / 2 //补偿项
//	     + cosf(alpha)*cosf(beta)*PM_TCP2PLATFROM;
//	H = (this->height_pid.fpDes - temp) * 0.001;//单位转换为M
  temp = cosf(beta)*sinf(alpha)* TOP_PLATFORM_length / 2 //补偿项
	     + cosf(alpha)*cosf(beta)*PM_TCP2PLATFROM;
	H = (this->height_pid.fpDes) * 0.001;//单位转换为M

  theta1 = (180.0 -(this->up_motor.fb_data.Pos_fb + INIT_POS_UP)) * RADIAN;
  theta2 = (180.0 -(this->rightdown_motor.fb_data.Pos_fb + INIT_POS_RIGHT)) * RADIAN;
  theta3 = (180.0 -(this->leftdown_motor.fb_data.Pos_fb + INIT_POS_LEFT)) * RADIAN;
//  theta1 = (180.0 -(this->up_motor.fb_data.Pos_fb + INIT_POS_test)) * RADIAN;
//  theta2 = (180.0 -(this->rightdown_motor.fb_data.Pos_fb + INIT_POS_test)) * RADIAN;
//  theta3 = (180.0 -(this->leftdown_motor.fb_data.Pos_fb + INIT_POS_test)) * RADIAN;
	
	
	fp32 fkp_a2,fkp_a5,fkp_a7,fkp_a8; 
	fp32 fkp_b2,fkp_b5,fkp_b7,fkp_b8; 
	fp32 fkp_c2,fkp_c5,fkp_c7,fkp_c8; 

  while(flag)
  {
	  this->pm_FKP.Iteration_k++;
	  //计算参数
	  fkp_a2 = -2*O2B1*(A1E-A1D1*cos(theta1)); 
    fkp_a5 = -2*A1D1*O2B1*sin(theta1);
    fkp_a7 = -2*A1D1*sin(theta1);
		temp = (A1E - A1D1*cos(theta1)) * (A1E - A1D1*cos(theta1));
    fkp_a8 = temp-B1D1*B1D1 + A1D1*A1D1*sin(theta1)*sin(theta1) + O2B1*O2B1;
 	  //A2D2 = A1D1 = A3D3
		//B2D2 = B1D1 = B3D3
    fkp_b2 = -2*O2B2*(A2E-A1D1*cos(theta2)); 
    fkp_b5 = -2*A1D1*O2B2*sin(theta2);
    fkp_b7 = -2*A1D1*sin(theta2);
		temp = (A2E - A1D1*cos(theta2))*(A2E - A1D1*cos(theta2));
    fkp_b8 = temp -B1D1*B1D1 + A1D1*A1D1*sin(theta2)*sin(theta2)+O2B2*O2B2; 	
    //O2B2 = O2B3  A3E = A2E  A1D1 = A3D3  B1D1 = B3D3
		fkp_c2 = -2*O2B2*(A2E-A1D1*cos(theta3)); 
    fkp_c5 =  2*A1D1*O2B2*sin(theta3);
    fkp_c7 = -2*A1D1*sin(theta3);
		temp = (A2E - A1D1*cos(theta3))*(A2E - A1D1*cos(theta3));
    fkp_c8 = temp -B1D1*B1D1 + A1D1*A1D1*sin(theta3)*sin(theta3)+ O2B2*O2B2;
    
		fp32 f1,f2,f3,f1a,f1h,f2b,f2h,f3b,f3h;
		
		f1 = fkp_a2*cos(alpha)+fkp_a4*sin(alpha)*H+fkp_a5*sin(alpha)+H*H+fkp_a7*H+fkp_a8;
    f2 = fkp_b2*cos(beta) +fkp_b4*sin(beta)*H +fkp_b5*sin(beta) +H*H+fkp_b7*H+fkp_b8;
    f3 = fkp_c2*cos(beta) +fkp_c4*sin(beta)*H +fkp_c5*sin(beta) +H*H+fkp_c7*H+fkp_c8;
 
    f1a = -fkp_a2*sin(alpha) + fkp_a4*H*cos(alpha) + fkp_a5*cos(alpha);
    f1h = fkp_a4*sin(alpha) + 2*H +fkp_a7;
    f2b = -fkp_b2*sin(beta) + fkp_b4*H*cos(beta) + fkp_b5*cos(beta);
    f2h = fkp_b4*sin(beta) + 2*H +fkp_b7; 
    f3b = -fkp_c2*sin(beta) + fkp_c4*H*cos(beta) + fkp_c5*cos(beta);
    f3h = fkp_c4*sin(beta) + 2*H +fkp_c7; 
    
		//计算dx
		fp32 d_x1,d_x2,d_x3;
		d_x1 = -(f1*f2b*f3h - f1*f3b*f2h + f2*f3b*f1h - f3*f2b*f1h)/(f1a*(f2b*f3h - f3b*f2h));
    d_x2 = -(f2*f3h - f3*f2h)/(f2b*f3h - f3b*f2h);   
    d_x3 =  (f2*f3b - f3*f2b)/(f2b*f3h - f3b*f2h);
		
		if(this->pm_FKP.Iteration_k >= this->pm_FKP.max_Iteration_k )//迭代超过一定次数
		{
			flag = 0;
		}
		//达到精度
    if((d_x1 < this->pm_FKP.accuracy) && (d_x2 < this->pm_FKP.accuracy) && (d_x3 < this->pm_FKP.accuracy))
		{
			 flag = 0;
       alpha = alpha + d_x1;
       beta = beta + d_x2;
       H = H + d_x3;			  
			
       this->alpha_pid.fpFB  = alpha * DEG;//转换为角度
       this->beta_pid.fpFB =   beta * DEG;
       temp = cosf(beta)*sinf(alpha)* TOP_PLATFORM_length / 2 //补偿项
	          + cosf(alpha)*cosf(beta)*PM_TCP2PLATFROM;
			 this->height_pid.fpFB = H*1000 + temp;
		}
		else
		{
       alpha = alpha + d_x1;
       beta = beta + d_x2;
       H = H + d_x3;  	
		}
		
  }
	
}	



//并联机构逆运动学
//给定目标 des_angle1; des_angle2; des_height; 求解三个电机角度，赋值给pos.des
void cParallelMechanism::ParallelMechanism_InverseKinematics(void)
{   
	  //输入参数限幅
    this->ParallelMechanism_para_protection();
	
	  fp32 OE_2,X_B2,X_B3,Y_B2,Y_B3,temp;
	  fp32 theta1 ,theta2;
	
	  theta1 = this->alpha_pid.fpDes * RADIAN;//转化为弧度
	  theta2 = this->beta_pid.fpDes * RADIAN;

	  //求B2 B3 XY坐标
	  temp = sinf(theta1) * cosf(theta2) * TOP_PLATFORM_length / 2 
	           + PM_TCP2PLATFROM * cosf(theta1) * cosf(theta2);
	  OE_2 = this->height_pid.fpDes - temp;
	
	  X_B2 = TOP_PLATFORM_LENGTH / 2 * cosf(theta2);
	  X_B3 = - X_B2;
	  Y_B2 = OE_2 + TOP_PLATFORM_LENGTH / 2 * sinf(theta2);
    Y_B3 = OE_2 - TOP_PLATFORM_LENGTH / 2 * sinf(theta2);

	  //求解theta1  up_motor
	  fp32 Y_B1,Z_B1;
    Y_B1 = SQRT3 / 2 * TOP_PLATFORM_LENGTH * cosf(theta1);
    Z_B1 = OE_2 + SQRT3 / 2 * TOP_PLATFORM_LENGTH * sinf(theta1);
	  temp = (Y_B1 - SQRT3 / 2 * BOTTOM_PLATFORM_LENGTH);
    fp32 A1B1 = sqrt(temp * temp + Z_B1 * Z_B1);
    fp32 angle_EA1B1 = (Z_B1)/(A1B1);
    angle_EA1B1 = asin(angle_EA1B1);
		temp = (A1B1 * A1B1  + PM_LENGTH_1*PM_LENGTH_1 - PM_LENGTH_2*PM_LENGTH_2);
    fp32 angle_B1A1D1 = temp / (2 * A1B1 * PM_LENGTH_1);
    angle_B1A1D1 = acos(angle_B1A1D1);
		temp = (angle_EA1B1 + angle_B1A1D1) * DEG;//单位转换为角度
		this->up_motor.crl_data.InvKine_Pos = temp;
    this->up_motor.crl_data.Pos_des = (180.0 - temp) - INIT_POS_UP;//转换到电机的坐标系
//    this->up_motor.crl_data.Pos_des = (180.0 - temp) - INIT_POS_test;//转换到电机的坐标系
   
	  //求解theta2  rightdown_motor
		temp = (X_B2 - BOTTOM_PLATFORM_LENGTH/2) * (X_B2 - BOTTOM_PLATFORM_LENGTH/2) + Y_B2 * Y_B2;
		fp32 A2B2 = sqrt(temp);
		temp = (A2B2 * A2B2 + BOTTOM_PLATFORM_LENGTH * BOTTOM_PLATFORM_LENGTH/4 - (X_B2 * X_B2  + Y_B2 * Y_B2));
    fp32 angle_EA2B2 = temp/(2 * A2B2 * BOTTOM_PLATFORM_LENGTH / 2);
    angle_EA2B2 = acos(angle_EA2B2);
		temp = (A2B2 * A2B2 + PM_LENGTH_1 * PM_LENGTH_1 - PM_LENGTH_2 * PM_LENGTH_2);
    fp32 angle_B2A2D2 = temp / (2 * A2B2 * PM_LENGTH_1);
    angle_B2A2D2 = acos(angle_B2A2D2);
		temp = (angle_EA2B2 + angle_B2A2D2) * DEG;//单位转换为角度
		this->rightdown_motor.crl_data.InvKine_Pos = temp;
    this->rightdown_motor.crl_data.Pos_des = (180.0 - temp) - INIT_POS_RIGHT;//转换到电机的坐标系
//    this->rightdown_motor.crl_data.Pos_des = (180.0 - temp) - INIT_POS_test;//转换到电机的坐标系

    //求解theta3  leftdown_motor		
		temp = (X_B3 + BOTTOM_PLATFORM_LENGTH / 2) * (X_B3 + BOTTOM_PLATFORM_LENGTH / 2) + Y_B3 * Y_B3;
		fp32 A3B3 = sqrt(temp);
		temp = (A3B3 * A3B3 + BOTTOM_PLATFORM_LENGTH * BOTTOM_PLATFORM_LENGTH / 4 - (X_B3 * X_B3 + Y_B3 * Y_B3));
    fp32 angle_EA3B3 = temp / ( 2 * A3B3 * BOTTOM_PLATFORM_LENGTH / 2);
    angle_EA3B3 = acos(angle_EA3B3);
		temp = (A3B3 * A3B3 + PM_LENGTH_1 * PM_LENGTH_1 - PM_LENGTH_2 * PM_LENGTH_2);
    fp32 angle_B3A3D3 = temp / (2 * A3B3 * PM_LENGTH_1);
    angle_B3A3D3 = acos(angle_B3A3D3);
		temp = (angle_EA3B3 + angle_B3A3D3) * DEG;//单位转换为角度
		this->leftdown_motor.crl_data.InvKine_Pos = temp;
		this->leftdown_motor.crl_data.Pos_des = (180.0 - temp) - INIT_POS_LEFT;	//转换到电机的坐标系	
//		this->leftdown_motor.crl_data.Pos_des = (180.0 - temp) - INIT_POS_test;	//转换到电机的坐标系	

}
//输入参数限幅
void cParallelMechanism::ParallelMechanism_para_protection(void)
{

  if(this->alpha_pid.fpDes > ANGLE_1_MAX)
	{
	  this->alpha_pid.fpDes = ANGLE_1_MAX;
	}
  if(this->alpha_pid.fpDes < - ANGLE_1_MAX)
	{
	  this->alpha_pid.fpDes = - ANGLE_1_MAX;
	}	
	
  if(this->beta_pid.fpDes > ANGLE_2_MAX)
	{
	  this->beta_pid.fpDes = ANGLE_2_MAX;
	}
  if(this->beta_pid.fpDes < - ANGLE_2_MAX)
	{
	  this->beta_pid.fpDes = - ANGLE_2_MAX;
	}

  if(this->height_pid.fpDes > HEIGHT_MAX)
	{
	  this->height_pid.fpDes = HEIGHT_MAX;
	}
  if(this->height_pid.fpDes <  HEIGHT_MIN)
	{
	  this->height_pid.fpDes =  HEIGHT_MIN;
	}	
	
}

void cParallelMechanism::ParallelMechanism_update(void)
{
   this->pre_des_angle1 = this->alpha_pid.fpDes;
   this->pre_des_angle2 = this->beta_pid.fpDes;
	 this->pre_des_height = this->height_pid.fpDes;
	 this->up_motor.crl_data.pre_Pos_des = this->up_motor.crl_data.Pos_des;
	 this->rightdown_motor.crl_data.pre_Pos_des = this->rightdown_motor.crl_data.Pos_des;
	 this->leftdown_motor.crl_data.pre_Pos_des = this->leftdown_motor.crl_data.Pos_des;

}

void cParallelMechanism::ParallelMechanism_crl_motor(void)
{
   PM_Motor.up_motor.trans_data(&can2_msg);
	 fdcan2.send_MSG(&can2_msg);
	 PM_Motor.up_motor.crl_data.send_cnt++;
	
   PM_Motor.rightdown_motor.trans_data(&can2_msg);
	 fdcan2.send_MSG(&can2_msg);
	 PM_Motor.rightdown_motor.crl_data.send_cnt++;
	
   PM_Motor.leftdown_motor.trans_data(&can2_msg);	
	 fdcan2.send_MSG(&can2_msg);
	 PM_Motor.leftdown_motor.crl_data.send_cnt++;
}

void cParallelMechanism::ParallelMechanism_enable_motor(void)
{
   PM_Motor.up_motor.enable_motor(&can2_msg);
	 fdcan2.send_MSG(&can2_msg);
	 PM_Motor.up_motor.motor_state = dm_Enable;
	
   PM_Motor.rightdown_motor.enable_motor(&can2_msg);
	 fdcan2.send_MSG(&can2_msg);
	 PM_Motor.rightdown_motor.motor_state = dm_Enable;
	
   PM_Motor.leftdown_motor.enable_motor(&can2_msg);	
	 fdcan2.send_MSG(&can2_msg);
	 PM_Motor.leftdown_motor.motor_state = dm_Enable;
}
void cParallelMechanism::ParallelMechanism_save_zeropoint(void)
{
   PM_Motor.up_motor.save_zero_point(&can2_msg);
	 fdcan2.send_MSG(&can2_msg);
	
   PM_Motor.rightdown_motor.save_zero_point(&can2_msg);
	 fdcan2.send_MSG(&can2_msg);
	
   PM_Motor.leftdown_motor.save_zero_point(&can2_msg);	
	 fdcan2.send_MSG(&can2_msg);
}
void cParallelMechanism::ParallelMechanism_disable_motor(void)
{
   PM_Motor.up_motor.disable_motor(&can2_msg);
	 fdcan2.send_MSG(&can2_msg);
	 PM_Motor.up_motor.motor_state = dm_Disable;
	
   PM_Motor.rightdown_motor.disable_motor(&can2_msg);
	 fdcan2.send_MSG(&can2_msg);
	 PM_Motor.rightdown_motor.motor_state = dm_Disable;
	
   PM_Motor.leftdown_motor.disable_motor(&can2_msg);	
	 fdcan2.send_MSG(&can2_msg);
	 PM_Motor.leftdown_motor.motor_state = dm_Disable;
}
void cParallelMechanism::ParallelMechanism_set_motor_PID(float kp , float kd , float Tff)
{
	 PM_Motor.up_motor.set_motor_PID( kp , kd , Tff );
	 PM_Motor.rightdown_motor.set_motor_PID( kp , kd , Tff );
	 PM_Motor.leftdown_motor.set_motor_PID( kp , kd , Tff );
}







