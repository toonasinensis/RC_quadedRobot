#include "navigation_algorithm.h"
#include "navigation_task.h"
#include "pid_algorithm.h"

float dead_zone = 0.0001;
cNav::cNav()
{
	this->state = NAV_INIT;//NAV_OFF;
	this->auto_path.pos_pid.x.fpKp = 6.0f;
	this->auto_path.pos_pid.x.fpUMax = 8000.0f;
	this->auto_path.pos_pid.y.fpKp = 6.0f;
	this->auto_path.pos_pid.y.fpUMax = 8000.0f;
	this->auto_path.pos_pid.w.fpKp = 5.0f;
	this->auto_path.pos_pid.w.fpUMax = 90*RADIAN;//̫����
	
	this->auto_path.velt_pid.x.fpKp = 0.0f;//35.0f;
	this->auto_path.velt_pid.x.fpKd = 0.0f;// 0.1f;
	this->auto_path.velt_pid.x.fpUMax = 0.0f;//10000.0f;
	this->auto_path.velt_pid.y.fpKp = 0.0f;//35.0f;
	this->auto_path.velt_pid.y.fpKd = 0.0f;//0.1f;
	this->auto_path.velt_pid.y.fpUMax = 0.0f;//10000.0f;
	this->auto_path.velt_pid.w.fpUMax = 0.0f;//10000.0f;
}

u8 RunMode = 0;

static chassis_run_state_e compare_min_angle(fp32 & forward, fp32 & backward, fp32 & feedback)
{
	if(fabs(forward - feedback) > 180)
	{
		if(forward > feedback)
		{
			forward -= 360;
		}
		else
		{
			forward += 360;
		}
	}
	if(fabs(backward - feedback) > 180)
	{
		if(backward > feedback)
		{
			backward -= 360;
		}
		else
		{
			backward += 360;
		}
	}
	if(fabs(backward - feedback) >= fabs(forward - feedback))
	{
		return FORWARD;
	}
	else
	{
		return BACKWARD;
	}
}


#if 0

/****************************************************************************************************
��������: void Handle_slip_ring_speed(ST_VECTOR *expect_robot_local_Velt,chassis_run_state_e state,
cMotor *turn_motor,cMotor *run_motor)
��������: ����������ϵ��ʾ�Ķ����ٶȴ��ݵ�PID��Ŀ��ֵ��
�������: 
���ز���: 
��   ע:	//ת�Ǵ����ǽ�x��ת������y��
****************************************************************************************************/

void Handle_slip_ring_speed(ST_VECTOR *expect_robot_local_Velt,
	chassis_run_state_e *state,cMotor *turn_motor,cMotor *run_motor)
{
		fp32 run_velt, turn_pos, turn_forward, turn_backward, turn_feedback;

	run_velt = (fp32)Geometric_mean(expect_robot_local_Velt->fpVy,expect_robot_local_Velt->fpVx)* 
						RUN_GEAR_RATIO / R_WHEEL;

	if(fabs(expect_robot_local_Velt->fpVy) < dead_zone)
	{
		if(expect_robot_local_Velt->fpVx > dead_zone)
			turn_pos = 0.0f * RADIAN;
		else if(expect_robot_local_Velt->fpVx < -dead_zone)
			turn_pos = 180.0f * RADIAN;
		else
			turn_pos = 0.0f * RADIAN;
	}
	else
	{
		if(fabs(expect_robot_local_Velt->fpVx) < dead_zone)
		{
			if(expect_robot_local_Velt->fpVy > dead_zone)
				turn_pos = 90.0f * RADIAN;
			else
				turn_pos = -90.0f * RADIAN;
		}
		else
		{
			turn_pos = (fp32)(atan2(expect_robot_local_Velt->fpVy, expect_robot_local_Velt->fpVx));
		}
	}

	// //�ı�Ƕȵ���Ƚ�λ���ٴ���,�Ƕ�ֵΪ-180��180
	// if(turn_pos >= -PI_2 && turn_pos <= PI)
	// {
	// 	turn_pos = turn_pos * 180.0f / PI - 90;//����ɽǶ��ٸı���Ƚ�λ��
	// }
	// else
	// {
	// 	turn_pos = 270 + turn_pos * 180.0f / PI;//����ɽǶ��ٸı���Ƚ�λ��
	// }
	
	if(turn_pos > 0)
	{
		turn_forward = turn_pos;
		turn_backward = turn_pos + 180;
	}
	else
	{
		turn_forward = turn_pos + 2 * 180;
		turn_backward = turn_pos + 180;
	}

	/*��Ӿ��񣬽��������ٶ�ȡ��*/
	//Wheel_C_Velt = -Wheel_C_Velt;
	turn_feedback = ((int32_t)turn_motor->pos_pid.fpFB + 720000) % 360 + 
		turn_motor->pos_pid.fpFB - (int32_t)turn_motor->pos_pid.fpFB;
	if(compare_min_angle(turn_forward, turn_backward, turn_feedback) == BACKWARD)
	{
		*state = BACKWARD;
		turn_motor->pos_pid.fpDes = turn_motor->pos_pid.fpFB +
			turn_backward - turn_feedback;
		run_motor->velt_pid.fpDes = -run_velt;
	}
	else
	{
		*state = FORWARD;
		turn_motor->pos_pid.fpDes = turn_motor->pos_pid.fpFB +
			turn_forward - turn_feedback;
		run_motor->velt_pid.fpDes = run_velt;
	}
}

#else
//�൱���Ѿ�֪��ÿ�����ӷ���ľֲ������µ��ٶ�ʸ����Ȼ���Ŀ��ֵ������н������ת����
void Handle_slip_ring_speed(ST_VECTOR *expect_robot_local_Velt,chassis_run_state_e *state,cMotor *turn_motor,cMotor *run_motor)
{
		fp32 run_velt, turn_pos, turn_forward, turn_backward, turn_feedback;
	Covert_coordinate(expect_robot_local_Velt,POLAR);

	run_velt = expect_robot_local_Velt->fpLength*RUN_GEAR_RATIO / R_WHEEL;//run_velt�ĵ�λ��rad/s
		turn_pos = 	expect_robot_local_Velt->fpthetha;			
	
	//��ת�Ǳ��0����360
	if(turn_pos > 0)
	{
		turn_forward = turn_pos;
		turn_backward = turn_pos + 180;
	}
	else
	{
		turn_forward = turn_pos + 2 * 180;
		turn_backward = turn_pos + 180;
	}

	/*��Ӿ��񣬽��������ٶ�ȡ��*/
	//Wheel_C_Velt = -Wheel_C_Velt;
	turn_feedback = ((int32_t)turn_motor->pos_pid.fpFB + 720000) % 360 + 
		turn_motor->pos_pid.fpFB - (int32_t)turn_motor->pos_pid.fpFB;
	if(compare_min_angle(turn_forward, turn_backward, turn_feedback) == BACKWARD)
	{
		*state = BACKWARD;
		turn_motor->pos_pid.fpDes = turn_motor->pos_pid.fpFB +
			turn_backward - turn_feedback;
		run_motor->velt_pid.fpDes = -run_velt;
	}
	else
	{
		*state = FORWARD;
		turn_motor->pos_pid.fpDes = turn_motor->pos_pid.fpFB +
			turn_forward - turn_feedback;
		run_motor->velt_pid.fpDes = run_velt;
	}
}

#endif
/****************************************************************************************************
��������: void cal_single_feed_forward(cNav *p_nav)
��������: ����һ�����ӵ�ǰ����������ת��ǰ��
�������: 
���ز���: 
��   ע:	ת�Ǵ����ǽ�x��ת������y��
****************************************************************************************************/

float cal_single_feed_forward(cNav *p_nav,chassis_run_state_e *state,
	ST_VECTOR *straight_velt,ST_VECTOR *pre_straight_velt,ST_VECTOR * d_straight_velt)
{
	ST_VECTOR expect_robot_local_Velt;
	fp32 fpQ, forfeed_current;
	fpQ = ConvertAngle(stRobot.stPot.fpPosQ * RADIAN_10);
	Concert_coorindnate(&p_nav->expect_robot_global_velt,&expect_robot_local_Velt, fpQ);
	straight_velt->fpLength = expect_robot_local_Velt.fpLength;
	d_straight_velt->fpLength = (straight_velt->fpLength - pre_straight_velt->fpLength) / RUN_GEAR_RATIO * R_WHEEL / 0.002f;//Ts
	if(*state == FORWARD)
	{
		forfeed_current = d_straight_velt->fpLength * K_UP_STRAIGHT +  START_CURRENT;
	}
	else
	{
		forfeed_current = -(d_straight_velt->fpLength * K_UP_STRAIGHT +  START_CURRENT);
	}
	*pre_straight_velt = *straight_velt;
	return forfeed_current;
}


/***************************************************���´���Ϊ4���л���***************************************************/
/***************************************************���´���Ϊ4���л���***************************************************/
/***************************************************���´���Ϊ4���л���***************************************************/
/***************************************************���´���Ϊ4���л���***************************************************/
/***************************************************���´���Ϊ4���л���***************************************************/
/***************************************************���´���Ϊ4���л���***************************************************/

//int a;
//void Call_curvature(ST_Trajectory *track)
//{
//	fp32 denominator ,numerator_x ,numerator_y;
//	denominator = track->acc_ref.fpVy*track->velt_ref.fpVx-track->velt_ref.fpVy*track->acc_ref.fpVx;
//	if(fabs(denominator)<EPS)
//	{
//		a = 1;
//	}
//	numerator_x = -track->velt_ref.fpVy*(track->velt_ref.fpVx*track->velt_ref.fpVx+ track->velt_ref.fpVy*track->velt_ref.fpVy);
//	numerator_y = track->velt_ref.fpVx *(track->velt_ref.fpVx*track->velt_ref.fpVx+ track->velt_ref.fpVy*track->velt_ref.fpVy);
//	
//	track->curvature.fpVx = numerator_x/denominator;
//	track->curvature.fpVy = numerator_y/denominator;
//	track->curvature.fpLength = Geometric_mean(track->curvature.fpVx,track->curvature.fpVy);
//}

/****************************************************************************************************
��������: Feedforward_by_curvature(cNav *p_nav)
��������: �����ʽ��㷽ʽ������4�����ֵ�ת�ٺ�ת��
�������: 
���ز���: 
��    ע:	X���Y = Z�����ʰ뾶Ϊ	Y���ٶ�ΪX
					V = W���R, ע���������������Ϊ0
****************************************************************************************************/

	ST_VECTOR expect_robot_local_curvature;
void Feedforward_by_curvature(cNav *p_nav,ST_Trajectory* track,chassis_velt_t *basic_local)
{
	chassis_velt_t pre_basic;
	 fp32 detal;
	ST_VECTOR R_leftup,R_rightup, R_rightdown, R_leftdown;
		chassis_velt_t velt_w,chassis_velt;

	ST_VECTOR pos_rightup = {A_EIGHT,L_EIGHT,0,0,0,CARTESIAN};
	ST_VECTOR pos_leftup	= {-A_EIGHT,L_EIGHT,0,0,0,CARTESIAN};
	ST_VECTOR pos_rightdown = {A_EIGHT,-L_EIGHT,0,0,0,CARTESIAN};
	ST_VECTOR pos_leftdown 	= {-A_EIGHT,-L_EIGHT,0,0,0,CARTESIAN};
	chassis_velt_t forefeed_velt;
	
	fp32 fpQ;
	//Call_curvature(track);
	if(track->state == CURVE)
	{
		track->velt_ref.fpLength =  Geometric_mean(track->velt_ref.fpVx,track->velt_ref.fpVy);
		track->w = track->velt_ref.fpLength / track->curvature.fpLength;
		track->w *= sign_judge(track->curvature.fpVy*track->velt_ref.fpVx - track->curvature.fpVx*track->velt_ref.fpVy);
		detal = pre_basic.rightdown.fpVx - basic_local->rightdown.fpVx;

		fpQ = ConvertAngle(stRobot.stPot.fpPosQ * RADIAN_10);
		Concert_coorindnate(&track->curvature,&expect_robot_local_curvature,fpQ);
		 //�����������ӵ��ٶ�
			Vector_minus(&pos_rightup,&expect_robot_local_curvature,&R_rightup);
			Vector_minus(&pos_leftup,&expect_robot_local_curvature,&R_leftup);
			Vector_minus(&pos_rightdown,&expect_robot_local_curvature,&R_rightdown);
			Vector_minus(&pos_leftdown,&expect_robot_local_curvature,&R_leftdown);
			
			Vector_cross(&R_rightup,   track->w,   &forefeed_velt.rightup   );
			Vector_cross(&R_leftup,    track->w,   &forefeed_velt.leftup    );
			Vector_cross(&R_rightdown, track->w,   &forefeed_velt.rightdown );
			Vector_cross(&R_leftdown,  track->w,   &forefeed_velt.leftdown  );
		basic_local->rightup = forefeed_velt.rightup    ;
	  basic_local->leftup = forefeed_velt.leftup      ;
	  basic_local->rightdown = forefeed_velt.rightdown;
	  basic_local->leftdown = forefeed_velt.leftdown  ;
		
	}
	else if(track->state == LINE)
	{
		
			fpQ = ConvertAngle(stRobot.stPot.fpPosQ * RADIAN_10);
	   	Concert_coorindnate(&track->velt_ref,&expect_robot_local_curvature,fpQ);
		 //�����������ӵ��ٶ�
			basic_local->rightup = expect_robot_local_curvature;
			basic_local->leftup = expect_robot_local_curvature;
			basic_local->rightdown = expect_robot_local_curvature;
			basic_local->leftdown = expect_robot_local_curvature;
	}
}
	


/****************************************************************************************************
��������: void SpeedDistribute_6DegreesOfFreedom_with_slip_ring(cNav *p_nav)
��������: ��PID����õ����ٶȺ�ǰ���ٶȼ������������ٶ�
�������: 
���ز���: 
****************************************************************************************************/

	ST_VECTOR expect_robot_local_Velt;

void SpeedDistribute_8DegreesOfFreedom_with_slip_ring(cNav *p_nav,chassis_velt_t *basic_local)
{
	
		chassis_velt_t velt_w,chassis_velt;

	ST_VECTOR pos_rightup   = {A_EIGHT,L_EIGHT,0,0,0,CARTESIAN};
	ST_VECTOR pos_leftup	= {-A_EIGHT,L_EIGHT,0,0,0,CARTESIAN};
	ST_VECTOR pos_rightdown = {A_EIGHT,-L_EIGHT,0,0,0,CARTESIAN};
	ST_VECTOR pos_leftdown 	= {-A_EIGHT,-L_EIGHT,0,0,0,CARTESIAN};
	fp32 fpQ, dead_zone;
	if(p_nav->state == NAV_MANUAL)
	{
		fpQ = 0;
	}
	else
	{
		fpQ = ConvertAngle(stRobot.stPot.fpPosQ * RADIAN_10);
	}
	
	Concert_coorindnate(&p_nav->expect_robot_global_velt,&expect_robot_local_Velt, fpQ);

	expect_robot_local_Velt.fpW = p_nav->expect_robot_global_velt.fpW;
	
	Vector_cross(&pos_rightup, expect_robot_local_Velt.fpW ,&velt_w.rightup);
	Vector_cross(&pos_rightdown, expect_robot_local_Velt.fpW ,&velt_w.rightdown);
	Vector_cross(&pos_leftup, expect_robot_local_Velt.fpW ,&velt_w.leftup);
	Vector_cross(&pos_leftdown, expect_robot_local_Velt.fpW ,&velt_w.leftdown);
	
	Vector_Plus(&(basic_local->rightup),&expect_robot_local_Velt,&chassis_velt.rightup);
	Vector_Plus(&(basic_local->leftup),&expect_robot_local_Velt,&chassis_velt.leftup);
	Vector_Plus(&(basic_local->rightdown),&expect_robot_local_Velt,&chassis_velt.rightdown);
	Vector_Plus(&(basic_local->leftdown),&expect_robot_local_Velt,&chassis_velt.leftdown);
 	
	Vector_Plus(&velt_w.rightup,  &chassis_velt.rightup, &chassis_velt.rightup);
	Vector_Plus(&velt_w.leftup,   &chassis_velt.leftup,  &chassis_velt.leftup);
	Vector_Plus(&velt_w.rightdown,&chassis_velt.rightdown,&chassis_velt.rightdown);
	Vector_Plus(&velt_w.leftdown, &chassis_velt.leftdown, &chassis_velt.leftdown);


	
	Handle_slip_ring_speed(&chassis_velt.rightup,     &chassis_run_state.rightup,  &chassis_turn.rightup_motor,   &chassis_run.rightup_motor);
	Handle_slip_ring_speed(&chassis_velt.leftup,      &chassis_run_state.leftup,   &chassis_turn.leftup_motor,    &chassis_run.leftup_motor);
	Handle_slip_ring_speed(&chassis_velt.leftdown,    &chassis_run_state.leftdown, &chassis_turn.leftdown_motor,  &chassis_run.leftdown_motor);
	Handle_slip_ring_speed(&chassis_velt.rightdown,   &chassis_run_state.rightdown,&chassis_turn.rightdown_motor,	&chassis_run.rightdown_motor);	

}


/****************************************************************************************************
��������: void tri_chassis_turn_with_slip_ring(fp32 up_des, fp32 leftdown_des, fp32 rightdown_des)
��������: ��������ת�Ƕ�
�������: -180~180��
���ز���: 
****************************************************************************************************/

void chassis_turn_with_slip_ring(fp32 rightup_des, fp32 rightdown_des, fp32 leftup_des, fp32 leftdown_des)
{
		fp32 R = 0.1;
	ST_VECTOR Velt_rightup,Velt_leftup,Velt_leftdown, Velt_rightdown;

	Velt_leftup.type = POLAR;
	Velt_rightup.type = POLAR;
	Velt_leftdown.type = POLAR;
	Velt_rightdown.type = POLAR;
	Velt_leftup.fpthetha = leftup_des;
	Velt_rightup.fpthetha = rightup_des;
	Velt_leftdown.fpthetha = leftdown_des;
	Velt_rightdown.fpthetha = rightdown_des;
	
	Handle_slip_ring_speed(&Velt_rightup,&chassis_run_state.rightup,&chassis_turn.rightup_motor,&chassis_run.rightup_motor);
	Handle_slip_ring_speed(&Velt_leftup,&chassis_run_state.leftup,&chassis_turn.leftup_motor,&chassis_run.leftup_motor);
	Handle_slip_ring_speed(&Velt_leftdown,&chassis_run_state.leftdown,&chassis_turn.leftdown_motor,&chassis_run.leftdown_motor);
	Handle_slip_ring_speed(&Velt_rightdown,&chassis_run_state.rightdown,&chassis_turn.rightdown_motor,&chassis_run.rightdown_motor);	
}


/*
��������ǰ��
*/
void cal_chassis_feed_forward(cNav *p_nav)
{
	chassis_velt_t straight_velt, turn_velt, d_straight_velt, d_turn_velt;
	static chassis_velt_t pre_straight_velt, pre_turn_velt;
	ST_VECTOR expect_robot_local_Velt;
	fp32 fpQ;
	fpQ = ConvertAngle(stRobot.stPot.fpPosQ * RADIAN_10);
	Concert_coorindnate(&p_nav->expect_robot_global_velt,&expect_robot_local_Velt, fpQ);

	chassis_run.rightup_motor.feed_forward_current =  cal_single_feed_forward(p_nav,&chassis_run_state.rightup,
	&expect_robot_local_Velt,&pre_straight_velt.rightup,& d_straight_velt.rightup);
	
	chassis_run.leftup_motor.feed_forward_current =  cal_single_feed_forward(p_nav,&chassis_run_state.leftup,
	&expect_robot_local_Velt,&pre_straight_velt.leftup,& d_straight_velt.leftup);
	
	chassis_run.leftdown_motor.feed_forward_current =  cal_single_feed_forward(p_nav,&chassis_run_state.leftdown,
	&expect_robot_local_Velt,&pre_straight_velt.leftdown,& d_straight_velt.leftdown);

	chassis_run.rightdown_motor.feed_forward_current =  cal_single_feed_forward(p_nav,&chassis_run_state.rightdown,
	&expect_robot_local_Velt,&pre_straight_velt.rightdown,& d_straight_velt.rightdown);
	/*��Ӿ��񣬽�ǰ��ȡ��*/
	//chassis_run.leftdown_motor.feed_forward_current = -chassis_run.leftdown_motor.feed_forward_current;
	
	pre_straight_velt = straight_velt;
	pre_turn_velt = turn_velt;
	
	/*��Ӿ��񣬽�ǰ��ȡ��*/
	//chassis_run.leftdown_motor.feed_forward_current = -chassis_run.leftdown_motor.feed_forward_current;
	
	pre_straight_velt = straight_velt;
	pre_turn_velt = turn_velt;
}



/***************************************************���´���Ϊ3��***************************************************/
/***************************************************���´���Ϊ3��***************************************************/
/***************************************************���´���Ϊ3��***************************************************/
/***************************************************���´���Ϊ3��***************************************************/
/***************************************************���´���Ϊ3��***************************************************/
/***************************************************���´���Ϊ3��***************************************************/

/****************************************************************************************************
��������: SpeedDistribute_6DegreesOfFreedom_with_slip_ring(cNav *p_nav)
��������: �����ʽ��㷽ʽ������3�����ֵ�ת�ٺ�ת��
�������: 
���ز���: 
��    ע:	X���Y = Z�����ʰ뾶Ϊ	Y���ٶ�ΪX
					V = W���R
****************************************************************************************************/

ST_VECTOR basic_local_velt_up,basic_local_velt_right,basic_local_velt_left;
void tri_Feedforward_by_curvature(ST_Trajectory*track,tri_chassis_velt_t *basic_local)
{
	ST_VECTOR pos_up = {0,TRI_CHASSIS_R,0,0,0,CARTESIAN};
	ST_VECTOR pos_right = {TRI_CHASSIS_R*COS_THETA,-TRI_CHASSIS_R*SIN_THETA,0,0,0,CARTESIAN};
	ST_VECTOR pos_left =  {-TRI_CHASSIS_R*COS_THETA,-TRI_CHASSIS_R*SIN_THETA,0,0,0,CARTESIAN};
	fp32 fpQ;
	ST_VECTOR R_up, R_right, R_left;
	ST_VECTOR expect_robot_local_curvature;

	track->w = track->velt_ref.fpLength / track->curvature.fpLength;
	track->w *= sign_judge(track->curvature.fpVy*track->velt_ref.fpVx - track->curvature.fpVx*track->velt_ref.fpVy);

	fpQ = ConvertAngle(stRobot.stPot.fpPosQ * RADIAN_10);
	Concert_coorindnate(&track->curvature,&expect_robot_local_curvature,fpQ);
	
	//�����������ӵ��ٶ�
		Vector_minus(&pos_up,&expect_robot_local_curvature,&R_up);
		Vector_minus(&pos_right,&expect_robot_local_curvature,&R_right);
		Vector_minus(&pos_left,&expect_robot_local_curvature,&R_left);
		basic_local->up.fpVx = - track->w* R_up.fpVy;
		basic_local->up.fpVy =  track->w* R_up.fpVx;
		basic_local->rightdown.fpVx = - track->w* R_right.fpVy;
		basic_local->rightdown.fpVy =  track->w* R_right.fpVx;
		basic_local->leftdown.fpVx = - track->w* R_left.fpVy;
		basic_local->leftdown.fpVy =  track->w* R_left.fpVx;
}

/****************************************************************************************************
��������: void SpeedDistribute_6DegreesOfFreedom_with_slip_ring(cNav *p_nav)
��������: ��PID����õ����ٶȺ�ǰ���ٶȼ������������ٶ�
�������: 
���ز���: 
****************************************************************************************************/


void SpeedDistribute_6DegreesOfFreedom_with_slip_ring(cNav *p_nav,tri_chassis_velt_t *basic_local)
{
	tri_chassis_velt_t velt_w,tri_chassis_velt;
//	ST_VECTOR pos_up   = {-TRI_CHASSIS_R,0,0,0,0,CARTESIAN};
//	ST_VECTOR pos_rightdown = {TRI_CHASSIS_R*cosf(PI/3),TRI_CHASSIS_R*sinf(PI/3),0,0,0,CARTESIAN};
//	ST_VECTOR pos_leftdown 	= {TRI_CHASSIS_R*cosf(-PI/3),TRI_CHASSIS_R*sinf(-PI/3),0,0,0,CARTESIAN};
	ST_VECTOR pos_leftdown   = {TRI_CHASSIS_R*cosf(7*PI/6),TRI_CHASSIS_R*sinf(7*PI/6),0,0,0,CARTESIAN};
	ST_VECTOR pos_up = {TRI_CHASSIS_R*cosf(PI/2),TRI_CHASSIS_R*sinf(PI/2),0,0,0,CARTESIAN};
	ST_VECTOR pos_rightdown 	= {TRI_CHASSIS_R*cosf(-PI/6),TRI_CHASSIS_R*sinf(-PI/6),0,0,0,CARTESIAN};
	fp32 fpQ, dead_zone;
	if(p_nav->state == NAV_MANUAL||p_nav->state==NAV_MANUAL_PATH)
	{
		fpQ = 0;
	}
	else
	{
		fpQ = ConvertAngle(stRobot.stPot.fpPosQ * RADIAN_10);
	}
	
	Concert_coorindnate(&p_nav->expect_robot_global_velt,&expect_robot_local_Velt, fpQ);

	expect_robot_local_Velt.fpW = p_nav->expect_robot_global_velt.fpW;
	
	Vector_cross(&pos_up, expect_robot_local_Velt.fpW ,&velt_w.up);
	Vector_cross(&pos_rightdown, expect_robot_local_Velt.fpW ,&velt_w.rightdown);
	Vector_cross(&pos_leftdown, expect_robot_local_Velt.fpW ,&velt_w.leftdown);
	
	Vector_Plus(&(basic_local->up),&expect_robot_local_Velt,&tri_chassis_velt.up);
	Vector_Plus(&(basic_local->rightdown),&expect_robot_local_Velt,&tri_chassis_velt.rightdown);
	Vector_Plus(&(basic_local->leftdown),&expect_robot_local_Velt,&tri_chassis_velt.leftdown);
 	
	Vector_Plus(&velt_w.up,       &tri_chassis_velt.up, &tri_chassis_velt.up);
	Vector_Plus(&velt_w.rightdown,&tri_chassis_velt.rightdown,&tri_chassis_velt.rightdown);
	Vector_Plus(&velt_w.leftdown, &tri_chassis_velt.leftdown, &tri_chassis_velt.leftdown);

	Handle_slip_ring_speed(&tri_chassis_velt.up,          &tri_chassis_run_state.up,  &tri_chassis_turn.up_motor,             &tri_chassis_run.up_motor);
	Handle_slip_ring_speed(&tri_chassis_velt.leftdown,    &tri_chassis_run_state.leftdown, &tri_chassis_turn.leftdown_motor,  &tri_chassis_run.leftdown_motor);
	Handle_slip_ring_speed(&tri_chassis_velt.rightdown,   &tri_chassis_run_state.rightdown,&tri_chassis_turn.rightdown_motor,	&tri_chassis_run.rightdown_motor);	
}

void Spin_Speed_distribute_6(cNav *p_nav,tri_chassis_velt_t *basic_local)
{
	tri_chassis_velt_t velt_w;
	ST_VECTOR pos_up = {0,TRI_CHASSIS_R,0,0,0,CARTESIAN};
	ST_VECTOR pos_rightdown = {TRI_CHASSIS_R*COS_THETA,-TRI_CHASSIS_R*SIN_THETA,0,0,0,CARTESIAN};
	ST_VECTOR pos_leftdown =  {-TRI_CHASSIS_R*COS_THETA,-TRI_CHASSIS_R*SIN_THETA,0,0,0,CARTESIAN};
	ST_VECTOR expect_robot_local_Velt,Velt_up,Velt_rightdown,Velt_leftdown;
	expect_robot_local_Velt.fpW = p_nav->expect_robot_global_velt.fpW;
	

	Vector_cross(&pos_rightdown, expect_robot_local_Velt.fpW ,&velt_w.rightdown);
	Vector_cross(&pos_up,        expect_robot_local_Velt.fpW ,&Velt_up);
	Vector_cross(&pos_leftdown,  expect_robot_local_Velt.fpW ,&velt_w.leftdown);
	
	Vector_Plus(&basic_local->up,&expect_robot_local_Velt,&Velt_up);
	Vector_Plus(&basic_local->rightdown,&expect_robot_local_Velt,&Velt_rightdown);
	Vector_Plus(&basic_local->leftdown,&expect_robot_local_Velt,&Velt_leftdown);
	
}
/****************************************************************************************************
��������: void PID_6DegreesOfFreedom_with_slip_ring(cNav *p_nav)
��������: ��PID����õ����ٶȷ����ٶ�
�������: 
���ز���: 
****************************************************************************************************/

void PID_6DegreesOfFreedom_with_slip_ring(cNav *p_nav,tri_chassis_velt_t *basic_local)
{
	fp32 fpQ;
	fp32 temp_vx;
	ST_VECTOR pos_up = {0,TRI_CHASSIS_R,0,0,0,CARTESIAN};
	ST_VECTOR pos_rightdown = {TRI_CHASSIS_R*COS_THETA,-TRI_CHASSIS_R*SIN_THETA,0,0,0,CARTESIAN};
	ST_VECTOR pos_leftdown =  {-TRI_CHASSIS_R*COS_THETA,-TRI_CHASSIS_R*SIN_THETA,0,0,0,CARTESIAN};
	ST_VECTOR expect_robot_local_Velt,Velt_up,Velt_rightdown,Velt_leftdown;
	fpQ = ConvertAngle(stRobot.stPot.fpPosQ * RADIAN_10);
	expect_robot_local_Velt.fpW = p_nav->expect_robot_global_velt.fpW;
	
  Spin_Speed_distribute_6(p_nav,basic_local);
	Concert_coorindnate(&p_nav->expect_robot_global_velt,&expect_robot_local_Velt, fpQ);
	
	Vector_Plus(&basic_local->up,&expect_robot_local_Velt,&Velt_up);
	Vector_Plus(&basic_local->rightdown,&expect_robot_local_Velt,&Velt_rightdown);
	Vector_Plus(&basic_local->leftdown,&expect_robot_local_Velt,&Velt_leftdown);
	
}

/****************************************************************************************************
��������: void tri_chassis_turn_with_slip_ring(fp32 up_des, fp32 leftdown_des, fp32 rightdown_des)
��������: ��������ת�Ƕ�
�������: 
���ز���: 
****************************************************************************************************/

void tri_chassis_turn_with_slip_ring(fp32 up_des, fp32 leftdown_des, fp32 rightdown_des)
{
	fp32 R = 0.1;
	ST_VECTOR Velt_up,Velt_leftdown, Velt_rightdown;
	Velt_up.type = POLAR;
	Velt_leftdown.type = POLAR;
	Velt_rightdown.type = POLAR;
	Velt_up.fpthetha = up_des;
	Velt_leftdown.fpthetha = leftdown_des;
	Velt_rightdown.fpthetha = rightdown_des;

	Handle_slip_ring_speed(&Velt_up,&tri_chassis_run_state.up,&tri_chassis_turn.up_motor,&tri_chassis_run.up_motor);
	Handle_slip_ring_speed(&Velt_leftdown,&tri_chassis_run_state.leftdown,&tri_chassis_turn.leftdown_motor,&tri_chassis_run.leftdown_motor);
	Handle_slip_ring_speed(&Velt_rightdown,&tri_chassis_run_state.rightdown,&tri_chassis_turn.rightdown_motor,&tri_chassis_run.rightdown_motor);	
}	

/****************************************************************************************************
��������: void cal_tri_chassis_feed_forward(cNav *p_nav)
��������: ��������ǰ��
�������: 
���ز���: 
****************************************************************************************************/

//void cal_tri_chassis_feed_forward(cNav *p_nav)
//{
//	tri_chassis_velt_t straight_velt, turn_velt, d_straight_velt, d_turn_velt;
//	static tri_chassis_velt_t pre_straight_velt, pre_turn_velt;
//	ST_VECTOR expect_robot_local_Velt;
//	fp32 fpQ;
//	fpQ = ConvertAngle(stRobot.stPot.fpPosQ * RADIAN_10);
//	Concert_coorindnate(&p_nav->expect_robot_global_velt,&expect_robot_local_Velt, fpQ);

//	tri_chassis_run.up_motor.feed_forward_current =  cal_single_feed_forward(p_nav,&tri_chassis_run_state.up,
//	&expect_robot_local_Velt,&pre_straight_velt.up,& d_straight_velt.up);

//	tri_chassis_run.leftdown_motor.feed_forward_current =  cal_single_feed_forward(p_nav,&tri_chassis_run_state.leftdown,
//	&expect_robot_local_Velt,&pre_straight_velt.leftdown,& d_straight_velt.leftdown);

//	tri_chassis_run.rightdown_motor.feed_forward_current =  cal_single_feed_forward(p_nav,&tri_chassis_run_state.rightdown,
//	&expect_robot_local_Velt,&pre_straight_velt.rightdown,& d_straight_velt.rightdown);
//	/*��Ӿ��񣬽�ǰ��ȡ��*/
//	//chassis_run.leftdown_motor.feed_forward_current = -chassis_run.leftdown_motor.feed_forward_current;
//	
//	pre_straight_velt = straight_velt;
//	pre_turn_velt = turn_velt;
//}

void cal_tri_chassis_feed_forward(cNav *p_nav)
{
	tri_chassis_type_t straight_velt, turn_velt, d_straight_velt, d_turn_velt;
	static tri_chassis_type_t pre_straight_velt, pre_turn_velt;
	ST_VECTOR expect_robot_local_Velt;
	fp32 fpQ;
	
	fpQ = ConvertAngle(stRobot.stPot.fpPosQ * RADIAN_10);

	//�����������µ��ٶ�
	expect_robot_local_Velt.fpVx = p_nav->auto_path.basic_velt.fpVx * cosf(fpQ) + 
													p_nav->auto_path.basic_velt.fpVy * sinf(fpQ);
	expect_robot_local_Velt.fpVy = -p_nav->auto_path.basic_velt.fpVx * sinf(fpQ) + 
													p_nav->auto_path.basic_velt.fpVy * cosf(fpQ);
	expect_robot_local_Velt.fpW = p_nav->auto_path.basic_velt.fpW;
	expect_robot_local_Velt.fpLength = sqrt(pow(expect_robot_local_Velt.fpVx, 2) + 
															pow(expect_robot_local_Velt.fpVy,2));

	//������ֱ�ߵ��ٶ�
	straight_velt.up = (fp32)(sqrt(pow((expect_robot_local_Velt.fpVy), 2) + 
								pow((expect_robot_local_Velt.fpVx), 2))) * RUN_GEAR_RATIO / R_WHEEL;
	straight_velt.leftdown = (fp32)(sqrt(pow((expect_robot_local_Velt.fpVy), 2) + 
								pow((expect_robot_local_Velt.fpVx), 2))) * RUN_GEAR_RATIO / R_WHEEL;
	straight_velt.rightdown = (fp32)(sqrt(pow((expect_robot_local_Velt.fpVy), 2) + 
								pow((expect_robot_local_Velt.fpVx), 2))) * RUN_GEAR_RATIO / R_WHEEL;

	d_straight_velt.up = (straight_velt.up - pre_straight_velt.up) / RUN_GEAR_RATIO * R_WHEEL / 0.002f;//Ts
	d_straight_velt.leftdown = (straight_velt.leftdown - pre_straight_velt.leftdown) / RUN_GEAR_RATIO * R_WHEEL / 0.002f;
	d_straight_velt.rightdown = (straight_velt.rightdown - pre_straight_velt.rightdown) / RUN_GEAR_RATIO * R_WHEEL / 0.002f;

  if(tri_chassis_run_state.up == FORWARD)
	{
		tri_chassis_run.up_motor.feed_forward_current = d_straight_velt.up * K_UP_STRAIGHT + 
													d_turn_velt.up * K_UP_TURN + START_CURRENT_UP;
	}
	else
	{
		tri_chassis_run.up_motor.feed_forward_current = -(d_straight_velt.up * K_UP_STRAIGHT + 
													d_turn_velt.up * K_UP_TURN + START_CURRENT_UP);
	}
	if(tri_chassis_run_state.leftdown == FORWARD)
	{
		tri_chassis_run.leftdown_motor.feed_forward_current = d_straight_velt.leftdown * K_LD_STRAIGHT + 
													d_turn_velt.leftdown * K_LD_TURN + START_CURRENT_LD;
	}
	else
	{
		tri_chassis_run.leftdown_motor.feed_forward_current = -(d_straight_velt.leftdown * K_LD_STRAIGHT + 
													d_turn_velt.leftdown * K_LD_TURN + START_CURRENT_LD);
	}
	
	if(tri_chassis_run_state.rightdown == FORWARD)
	{
		tri_chassis_run.rightdown_motor.feed_forward_current = d_straight_velt.rightdown * K_RD_STRAIGHT + 
													d_turn_velt.rightdown * K_RD_TURN + START_CURRENT_RD;
	}
	else
	{
		tri_chassis_run.rightdown_motor.feed_forward_current = -(d_straight_velt.rightdown * K_RD_STRAIGHT + 
													d_turn_velt.rightdown * K_RD_TURN + START_CURRENT_RD);
	}

	
	/*��Ӿ��񣬽�ǰ��ȡ��*/
	//chassis_run.leftdown_motor.feed_forward_current = -chassis_run.leftdown_motor.feed_forward_current;
	
	pre_straight_velt = straight_velt;
	pre_turn_velt = turn_velt;
}

void turn_Q(fp32& des,fp32& fb)
{
	float des_forward,des_backward,fb_feedback;
	des=((int32_t)des + 720000) % 360 + 
		des - (int32_t)des;
	if(des>180)
	{
		des_forward = des - 360;
	}
	else
	{
		des_forward = des;
	}
	
	fb_feedback=((int32_t)fb + 720000) % 360 + 
		fb - (int32_t)fb;
	if(fb_feedback>180)
	{
		fb_feedback = fb_feedback - 360;
	}
	else
	{
		fb_feedback = fb_feedback;
	}
	
		des = fb +
			des_forward - fb_feedback;
}

/**********************************************���´���Ϊ4ȫ���ֵ���***************************************************/
/**********************************************���´���Ϊ4ȫ���ֵ���***************************************************/
/**********************************************���´���Ϊ4ȫ���ֵ���***************************************************/
/**********************************************���´���Ϊ4ȫ���ֵ���***************************************************/
/**********************************************���´���Ϊ4ȫ���ֵ���***************************************************/
/**********************************************���´���Ϊ4ȫ���ֵ���***************************************************/
/****************************************************************************************************
��������: void PID_4DegreesOfFreedom_Omni_Wheel(cNav *p_nav,chassis_velt_t *basic_local)
��������: ��PID����õ����ٶȷ����ٶ�
�������: 
���ز���: 
****************************************************************************************************/

//	ST_VECTOR expect_robot_local_Velt;
	chassis_velt_t velt_w,chassis_velt;
void PID_4DegreesOfFreedom_Omni_Wheel(cNav *p_nav)
{	

	ST_VECTOR pos_rightup   = {A_EIGHT,L_EIGHT,0,0,0,CARTESIAN};
	ST_VECTOR pos_leftup	  = {-A_EIGHT,L_EIGHT,0,0,0,CARTESIAN};
	ST_VECTOR pos_rightdown = {A_EIGHT,-L_EIGHT,0,0,0,CARTESIAN};
	ST_VECTOR pos_leftdown 	= {-A_EIGHT,-L_EIGHT,0,0,0,CARTESIAN};
	ST_VECTOR pos_rightup_2,pos_leftup_2,pos_rightdown_2,pos_leftdown_2;
	fp32 fpQ,L_rightup,L_leftup,L_rightdown,L_leftdown;
	
//	if(p_nav->state == NAV_MANUAL||p_nav->state==NAV_MANUAL_PATH)
	if(p_nav->state==NAV_MANUAL_PATH)
	{
		fpQ = 0;
	}
	else
	{
		fpQ = ConvertAngle(stRobot.stPot.fpPosQ * RADIAN_10);
//		fpQ = ConvertAngle(nav.auto_path.pos_pid.w.fpDes * RADIAN);
	}
	
	Concert_coorindnate(&p_nav->expect_robot_global_velt,&expect_robot_local_Velt, fpQ);
	Covert_coordinate(&pos_rightup,CARTESIAN);
	Covert_coordinate(&pos_leftup,CARTESIAN);
	Covert_coordinate(&pos_rightup,CARTESIAN);
	Covert_coordinate(&pos_rightup,CARTESIAN);
	
	Vector_cross(&pos_rightup, expect_robot_local_Velt.fpW ,&velt_w.rightup);
	Vector_cross(&pos_rightdown, expect_robot_local_Velt.fpW ,&velt_w.rightdown);
	Vector_cross(&pos_leftup, expect_robot_local_Velt.fpW ,&velt_w.leftup);
	Vector_cross(&pos_leftdown, expect_robot_local_Velt.fpW ,&velt_w.leftdown);
	
	Vector_Turn(&pos_rightup,  &pos_rightup_2,  90.0);
	Vector_Turn(&pos_rightdown,&pos_rightdown_2,90.0);
	Vector_Turn(&pos_leftup,   &pos_leftup_2,   90.0);
	Vector_Turn(&pos_leftdown, &pos_leftdown_2, 90.0);

  Vector_Projection_BonA(&pos_rightup_2,  &expect_robot_local_Velt,&chassis_velt.rightup); 	
  Vector_Projection_BonA(&pos_rightdown_2,&expect_robot_local_Velt,&chassis_velt.rightdown); 	
  Vector_Projection_BonA(&pos_leftup_2,   &expect_robot_local_Velt,&chassis_velt.leftup); 	
  Vector_Projection_BonA(&pos_leftdown_2, &expect_robot_local_Velt,&chassis_velt.leftdown); 		
	
	Vector_Plus(&velt_w.rightup,  &chassis_velt.rightup, &chassis_velt.rightup);
	Vector_Plus(&velt_w.leftup,   &chassis_velt.leftup,  &chassis_velt.leftup);
	Vector_Plus(&velt_w.rightdown,&chassis_velt.rightdown,&chassis_velt.rightdown);
	Vector_Plus(&velt_w.leftdown, &chassis_velt.leftdown, &chassis_velt.leftdown);
	
	//��ϵ�ֲ��ٶ���_2 �ϵ�ͶӰ���з���
	Vector_Projection_BonA_2(&pos_rightup_2  ,&chassis_velt.rightup,  &L_rightup);
	Vector_Projection_BonA_2(&pos_rightdown_2,&chassis_velt.rightdown,&L_rightdown);
	Vector_Projection_BonA_2(&pos_leftup_2   ,&chassis_velt.leftup,   &L_leftup);
	Vector_Projection_BonA_2(&pos_leftdown_2 ,&chassis_velt.leftdown, &L_leftdown);
  
	chassis_run.rightup_motor  .velt_pid.fpDes = L_rightup   * RUN_GEAR_RATIO / R_WHEEL;
	chassis_run.leftup_motor   .velt_pid.fpDes = L_leftup    * RUN_GEAR_RATIO / R_WHEEL;
	chassis_run.leftdown_motor .velt_pid.fpDes = L_leftdown  * RUN_GEAR_RATIO / R_WHEEL;
	chassis_run.rightdown_motor.velt_pid.fpDes = L_rightdown * RUN_GEAR_RATIO / R_WHEEL;
	
}

