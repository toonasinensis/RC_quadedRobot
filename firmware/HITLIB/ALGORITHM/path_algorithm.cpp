#include "path_algorithm.h"

fp32 path1_acc, path1_adjust_time, path1_velt_max;
ST_POS path1_des_point, path1_now_point;

//普通路径的依赖变量
float flag_record = 0;
int flag_path_end = 0;
float StartX, StartY, StartQ, DELTA_X, DELTA_Y, DELTA_Q, t_run, Ts = 0.001;
fp32 t1, t2, t3, t4, t5, t6, t7, t8;

// 1-6 取右侧红球打塔 -6-1 取左侧篮球打R2
int times_path_left = 6;
int times_path_right = 1;

POINT point_end(2000, 2000, 90);
VandA vanda(1000, 1000, 1000);

//直线x+方向测试
static void path_x_test(cNav *p_nav)
{
//	static fp32 S;
	static fp32 S1, S2, S3;
	static fp32 A_up;
	static fp32 A_down;
	// fp32 vision_angle;
	//  static fp32 S_x,S_y;
//	static fp32 R;
	static fp32 Vmax;
	// VISION_POT.fpPosQ = WAIT_angle;
	if (flag_record) //初始化加速度，运动时间，位移etc
	{
		A_up = 4000;   //-4500;
		A_down = 4000; //-4500;
		Vmax = 4000;
		DELTA_X = 4000;
		DELTA_Y = 0;

		StartX = p_nav->auto_path.pos_pid.x.fpFB;
		StartY = p_nav->auto_path.pos_pid.y.fpFB;
		StartQ = p_nav->auto_path.pos_pid.w.fpFB;

		S1 = Vmax * Vmax / (2 * A_up);
		S3 = Vmax * Vmax / (2 * A_down);
		S2 = DELTA_X - S1 - S3;
		t1 = Vmax / A_up;
		t3 = Vmax / A_down;
		t2 = S2 / Vmax;

		p_nav->auto_path.pos_pid.x.fpSumE = 0;
		p_nav->auto_path.pos_pid.y.fpSumE = 0;
		p_nav->auto_path.pos_pid.w.fpSumE = 0;
		flag_record = 0;
	}

	t_run = Ts * p_nav->auto_path.run_time;
	if (t_run < t1)
	{
		p_nav->auto_path.pos_pid.x.fpDes = StartX + 0.5f * A_up * t_run * t_run;
		p_nav->auto_path.pos_pid.y.fpDes = StartY;
		p_nav->auto_path.basic_velt.fpVx = A_up * t_run;
		p_nav->auto_path.basic_velt.fpVy = 0;
		p_nav->auto_path.pos_pid.w.fpDes = StartQ;
		p_nav->auto_path.basic_velt.fpW = 0;
		// p_nav->auto_path.acceleration = A_up;
	}
	else if (t_run < t1 + t2)
	{
		p_nav->auto_path.pos_pid.x.fpDes = StartX + S1 + Vmax * (t_run - t1);
		p_nav->auto_path.pos_pid.y.fpDes = StartY;
		p_nav->auto_path.basic_velt.fpVx = Vmax;
		p_nav->auto_path.basic_velt.fpVy = 0;
		// p_nav->auto_path.acceleration    = 0;
		p_nav->auto_path.pos_pid.w.fpDes = StartQ;
		p_nav->auto_path.basic_velt.fpW = 0;
	}
	else if (t_run < t1 + t2 + t3)
	{
		p_nav->auto_path.pos_pid.x.fpDes = StartX + DELTA_X - 0.5f * A_down * (t1 + t2 + t3 - t_run) * (t1 + t2 + t3 - t_run);
		p_nav->auto_path.pos_pid.y.fpDes = StartY;
		p_nav->auto_path.basic_velt.fpVx = Vmax - A_down * (t_run - t1 - t2);
		p_nav->auto_path.basic_velt.fpVy = 0;
		// p_nav->auto_path.acceleration    = 0;
		p_nav->auto_path.pos_pid.w.fpDes = StartQ;
		p_nav->auto_path.basic_velt.fpW = 0;
	}
	else if (t_run < t1 + t2 + t3 + 0.1f)
	{
		p_nav->auto_path.pos_pid.x.fpDes = StartX + DELTA_X;
		p_nav->auto_path.pos_pid.y.fpDes = StartY + DELTA_Y;
		p_nav->auto_path.basic_velt.fpVx = 0;
		p_nav->auto_path.basic_velt.fpVy = 0;
		// p_nav->auto_path.acceleration    = 0;
		p_nav->auto_path.pos_pid.w.fpDes = StartQ;
		p_nav->auto_path.basic_velt.fpW = 0;
	}
	else
	{
		p_nav->auto_path.run_time = 0;
		p_nav->state = NAV_STOP;

		p_nav->auto_path.pos_pid.x.fpSumE = 0;
		p_nav->auto_path.pos_pid.y.fpSumE = 0;
		p_nav->auto_path.pos_pid.w.fpSumE = 0;
	}
	// p_nav->auto_path.basic_velt.fpW *= 0.5f;
}

//直线y+方向测试
static void path_y_test(cNav *p_nav)
{
//	static fp32 S;
	static fp32 S1, S2, S3;
	static fp32 A_up;
	static fp32 A_down;
	// fp32 vision_angle;
	//  static fp32 S_x,S_y;
//	static fp32 R;
	static fp32 Vmax;
	// VISION_POT.fpPosQ = WAIT_angle;
	if (flag_record) //初始化加速度，运动时间，位移etc
	{
		A_up = 4000;   //-4500;
		A_down = 4000; //-4500;
		Vmax = 4000;
		DELTA_X = 0;
		DELTA_Y = 4000;

		StartX = p_nav->auto_path.pos_pid.x.fpFB;
		StartY = p_nav->auto_path.pos_pid.y.fpFB;
		StartQ = p_nav->auto_path.pos_pid.w.fpFB;

		S1 = Vmax * Vmax / (2 * A_up);
		S3 = Vmax * Vmax / (2 * A_down);
		S2 = DELTA_Y - S1 - S3;
		t1 = Vmax / A_up;
		t3 = Vmax / A_down;
		t2 = S2 / Vmax;

		p_nav->auto_path.pos_pid.x.fpSumE = 0;
		p_nav->auto_path.pos_pid.y.fpSumE = 0;
		p_nav->auto_path.pos_pid.w.fpSumE = 0;
		flag_record = 0;
	}

	t_run = Ts * p_nav->auto_path.run_time;
	if (t_run < t1)
	{
		p_nav->auto_path.pos_pid.x.fpDes = StartX;
		p_nav->auto_path.pos_pid.y.fpDes = StartY + 0.5f * A_up * t_run * t_run;
		p_nav->auto_path.basic_velt.fpVx = 0;
		p_nav->auto_path.basic_velt.fpVy = A_up * t_run;
		p_nav->auto_path.pos_pid.w.fpDes = StartQ;
		p_nav->auto_path.basic_velt.fpW = 0;
		// p_nav->auto_path.acceleration = A_up;
	}
	else if (t_run < t1 + t2)
	{
		p_nav->auto_path.pos_pid.x.fpDes = StartX;
		p_nav->auto_path.pos_pid.y.fpDes = StartY + S1 + Vmax * (t_run - t1);
		p_nav->auto_path.basic_velt.fpVx = 0;
		p_nav->auto_path.basic_velt.fpVy = Vmax;
		// p_nav->auto_path.acceleration    = 0;
		p_nav->auto_path.pos_pid.w.fpDes = StartQ;
		p_nav->auto_path.basic_velt.fpW = 0;
	}
	else if (t_run < t1 + t2 + t3)
	{
		p_nav->auto_path.pos_pid.x.fpDes = StartX;
		p_nav->auto_path.pos_pid.y.fpDes = StartY + DELTA_Y - 0.5f * A_down * (t1 + t2 + t3 - t_run) * (t1 + t2 + t3 - t_run);
		p_nav->auto_path.basic_velt.fpVx = 0;
		p_nav->auto_path.basic_velt.fpVy = Vmax - A_down * (t_run - t1 - t2);
		// p_nav->auto_path.acceleration    = 0;
		p_nav->auto_path.pos_pid.w.fpDes = StartQ;
		p_nav->auto_path.basic_velt.fpW = 0;
	}
	else if (t_run < t1 + t2 + t3 + 0.1f)
	{
		p_nav->auto_path.pos_pid.x.fpDes = StartX + DELTA_X;
		p_nav->auto_path.pos_pid.y.fpDes = StartY + DELTA_Y;
		p_nav->auto_path.basic_velt.fpVx = 0;
		p_nav->auto_path.basic_velt.fpVy = 0;
		// p_nav->auto_path.acceleration    = 0;
		p_nav->auto_path.pos_pid.w.fpDes = StartQ;
		p_nav->auto_path.basic_velt.fpW = 0;
	}
	else
	{
		p_nav->auto_path.run_time = 0;
		p_nav->state = NAV_STOP;

		p_nav->auto_path.pos_pid.x.fpSumE = 0;
		p_nav->auto_path.pos_pid.y.fpSumE = 0;
		p_nav->auto_path.pos_pid.w.fpSumE = 0;
	}
	// p_nav->auto_path.basic_velt.fpW *= 0.5f;
}

void path_straight(cNav *p_nav)
{
	static fp32 S;
	static fp32 S1, S2, S3;
	static fp32 A_up;
	static fp32 A_down;
	static fp32 Vmax;
	static fp32 alpha;
	static fp32 S_move;
	static fp32 V_move;
	static fp32 W_move;

	if (flag_record) //初始化加速度，运动时间，位移etc
	{
		//取块3000,1000,2000
		//回交接点3000,3000,2000
		//跑路径3000,3000,3000
		A_up = vanda.A_up;	   // 3000;
		A_down = vanda.A_down; // 3000;
		Vmax = vanda.Vmax;	   // 3000;

		StartX = p_nav->auto_path.pos_pid.x.fpFB;
		StartY = p_nav->auto_path.pos_pid.y.fpFB;
		StartQ = p_nav->auto_path.pos_pid.w.fpFB;

		//类似转向的控制
		turn_Q(point_end.q_end, StartQ);

		DELTA_X = point_end.x_end - StartX;
		DELTA_Y = point_end.y_end - StartY;
		DELTA_Q = point_end.q_end - StartQ;

		//可能要调整绝对式Q大小
		if (fabs(DELTA_Q) < 5)
		{
			StartQ = point_end.q_end;
			DELTA_Q = 0;
		}

		S = Geometric_mean(DELTA_X, DELTA_Y);
		alpha = atan2f(DELTA_Y, DELTA_X);

		if (S <= (Vmax * Vmax / (2 * A_up) + Vmax * Vmax / (2 * A_down)))
		{
			S1 = S / (A_up + A_down) * A_down;
			S2 = 0;
			S3 = S / (A_up + A_down) * A_up;
			t1 = sqrt(2 * S1 / A_up);
			t2 = 0;
			t3 = sqrt(2 * S3 / A_down);
			Vmax = A_up * t1;
		}
		else
		{
			t1 = Vmax / A_up;
			S1 = Vmax * Vmax / (2 * A_up);
			t3 = Vmax / A_down;
			S3 = Vmax * Vmax / (2 * A_down);
			S2 = S - S1 - S3;
			t2 = S2 / Vmax;
		}

		W_move = DELTA_Q / (0.8f * (t1 + t2 + t3));
		if (fabs(W_move) >= 90)
		{
			W_move = 0;
			DELTA_Q = 0;
		}

		p_nav->auto_path.pos_pid.x.fpSumE = 0;
		p_nav->auto_path.pos_pid.y.fpSumE = 0;
		p_nav->auto_path.pos_pid.w.fpSumE = 0;
		flag_record = 0;
	}

	t_run = Ts * p_nav->auto_path.run_time;
	if (t_run < 0.8f * (t1 + t2 + t3))
	{
		p_nav->auto_path.pos_pid.w.fpDes = StartQ + W_move * t_run;
		p_nav->auto_path.basic_velt.fpW = W_move;
	}
	else if (t_run < t1 + t2 + t3 + 0.1f)
	{
		p_nav->auto_path.pos_pid.w.fpDes = StartQ + DELTA_Q;
		p_nav->auto_path.basic_velt.fpW = 0;
	}
	else
	{
		p_nav->auto_path.pos_pid.w.fpSumE = 0;
	}

	if (t_run < t1)
	{
		S_move = 0.5f * A_up * t_run * t_run;
		V_move = A_up * t_run;
		p_nav->auto_path.pos_pid.x.fpDes = StartX + S_move * cosf(alpha);
		p_nav->auto_path.pos_pid.y.fpDes = StartY + S_move * sinf(alpha);
		p_nav->auto_path.basic_velt.fpVx = V_move * cosf(alpha);
		p_nav->auto_path.basic_velt.fpVy = V_move * sinf(alpha);
	}
	else if (t_run < t1 + t2)
	{
		S_move = S1 + Vmax * (t_run - t1);
		V_move = Vmax;
		p_nav->auto_path.pos_pid.x.fpDes = StartX + S_move * cosf(alpha);
		p_nav->auto_path.pos_pid.y.fpDes = StartY + S_move * sinf(alpha);
		p_nav->auto_path.basic_velt.fpVx = V_move * cosf(alpha);
		p_nav->auto_path.basic_velt.fpVy = V_move * sinf(alpha);
	}
	else if (t_run < t1 + t2 + t3)
	{
		S_move = S1 + S2 + Vmax * (t_run - t1 - t2) - 0.5f * A_down * (t_run - t1 - t2) * (t_run - t1 - t2);
		V_move = Vmax - A_down * (t_run - t1 - t2);
		p_nav->auto_path.pos_pid.x.fpDes = StartX + S_move * cosf(alpha);
		p_nav->auto_path.pos_pid.y.fpDes = StartY + S_move * sinf(alpha);
		p_nav->auto_path.basic_velt.fpVx = V_move * cosf(alpha);
		p_nav->auto_path.basic_velt.fpVy = V_move * sinf(alpha);
	}
	else if (t_run < t1 + t2 + t3 + 0.1f)
	{
		p_nav->auto_path.pos_pid.x.fpDes = StartX + DELTA_X;
		p_nav->auto_path.pos_pid.y.fpDes = StartY + DELTA_Y;
		p_nav->auto_path.basic_velt.fpVx = 0;
		p_nav->auto_path.basic_velt.fpVy = 0;
	}
	else
	{
		p_nav->auto_path.run_time = 0;
		p_nav->state = NAV_HIT;

		p_nav->auto_path.pos_pid.x.fpSumE = 0;
		p_nav->auto_path.pos_pid.y.fpSumE = 0;
	}
}

static void path_curve(cNav *p_nav)
{
	static fp32 A_up_x, A_up_y;
	static fp32 A_down_x, A_down_y;
//	static fp32 R;
	static fp32 Vmax_x;
	static fp32 Vmax_y;
	static fp32 Vw;
	static fp32 X1, X2, X3;
	static fp32 Y1, Y2, Y3;
//	static fp32 END_Q;

	if (flag_record)
	{
		A_up_x = -3000;
		A_down_x = -3000;
		A_up_y = -3000;
		A_down_y = -3000;

		Vmax_x = -3000;
		Vmax_y = -3000;

		StartX = p_nav->auto_path.pos_pid.x.fpFB;
		StartY = p_nav->auto_path.pos_pid.y.fpFB;
		StartQ = p_nav->auto_path.pos_pid.w.fpFB;

		turn_Q(point_end.q_end, StartQ);
		
		DELTA_X = point_end.x_end - StartX;
		DELTA_Y = point_end.y_end - StartY;
		DELTA_Q = point_end.q_end - StartQ;

		if (fabs(DELTA_X) <= fabs((Vmax_x * Vmax_x / (2 * A_up_x) + Vmax_x * Vmax_x / (2 * A_down_x))))
		{
			X1 = DELTA_X / (A_up_x + A_down_x) * A_down_x;
			X2 = 0;
			X3 = DELTA_X / (A_up_x + A_down_x) * A_up_x;
			t1 = sqrt(2 * X1 / A_up_x);
			t2 = 0;
			t3 = sqrt(2 * X3 / A_down_x);
			Vmax_x = A_up_x * t1;
		}
		else
		{
			t1 = Vmax_x / A_up_x;
			X1 = Vmax_x * Vmax_x / (2 * A_up_x);
			t3 = Vmax_x / A_down_x;
			X3 = Vmax_x * Vmax_x / (2 * A_down_x);
			X2 = DELTA_X - X1 - X3;
			t2 = X2 / Vmax_x;
		}

		if (fabs(DELTA_Y) <= fabs((Vmax_y * Vmax_y / (2 * A_up_y) + Vmax_y * Vmax_y / (2 * A_down_y))))
		{
			Y1 = DELTA_Y / (A_up_y + A_down_y) * A_down_y;
			Y2 = 0;
			Y3 = DELTA_Y / (A_up_y + A_down_y) * A_up_y;
			t4 = sqrt(2 * Y1 / A_up_y);
			t5 = 0;
			t6 = sqrt(2 * Y3 / A_down_y);
			Vmax_y = A_up_y * t4;
		}
		else
		{
			t4 = Vmax_y / A_up_y;
			Y1 = Vmax_y * Vmax_y / (2 * A_up_y);
			t6 = Vmax_y / A_down_y;
			Y3 = Vmax_y * Vmax_y / (2 * A_down_y);
			Y2 = DELTA_Y - Y1 - Y3;
			t5 = Y2 / Vmax_y;
		}

		Vw = DELTA_Q / (t1 + t2 + t4 + t5 + t6);

		p_nav->auto_path.pos_pid.x.fpSumE = 0;
		p_nav->auto_path.pos_pid.y.fpSumE = 0;
		p_nav->auto_path.pos_pid.w.fpSumE = 0;
		flag_record = 0;
	}

	t_run = Ts * p_nav->auto_path.run_time;

	if (t_run < t1 + t2 + t4 + t5 + t6)
	{
		p_nav->auto_path.pos_pid.w.fpDes = StartQ + Vw * t_run;
		p_nav->auto_path.basic_velt.fpW = Vw;
	}
	else
	{
		p_nav->auto_path.pos_pid.w.fpDes = 0;
		p_nav->auto_path.basic_velt.fpW = 0;
	}

	if (t_run < t1)
	{
		p_nav->auto_path.pos_pid.x.fpDes = StartX + 0.5f * A_up_x * t_run * t_run;
		p_nav->auto_path.basic_velt.fpVx = A_up_x * t_run;
	}
	else if (t_run < t1 + t2)
	{
		p_nav->auto_path.pos_pid.x.fpDes = StartX + 0.5f * A_up_x * t1 * t1 + Vmax_x * (t_run - t1);
		p_nav->auto_path.basic_velt.fpVx = Vmax_x;
	}
	else if (t_run < t1 + t2 + t3)
	{
		p_nav->auto_path.pos_pid.x.fpDes = StartX + 0.5f * A_up_x * t1 * t1 + Vmax_x * t2 + Vmax_x * (t_run - t1 - t2) - 0.5f * A_down_x * (t_run - t1 - t2) * (t_run - t1 - t2);
		p_nav->auto_path.basic_velt.fpVx = Vmax_x - A_down_x * (t_run - t1 - t2);
	}
	else
	{
		p_nav->auto_path.pos_pid.x.fpDes = StartX + DELTA_X;
		p_nav->auto_path.basic_velt.fpVx = 0;
	}

	if (t_run < t1 + t2)
	{
		p_nav->auto_path.pos_pid.y.fpDes = StartY;
		p_nav->auto_path.basic_velt.fpVy = 0;
	}
	else if (t_run < t1 + t2 + t4)
	{
		p_nav->auto_path.pos_pid.y.fpDes = StartY + 0.5f * A_up_y * (t_run - t1 - t2) * (t_run - t1 - t2);
		p_nav->auto_path.basic_velt.fpVy = A_up_y * (t_run - t1 - t2);
	}
	else if (t_run < t1 + t2 + t4 + t5)
	{
		p_nav->auto_path.pos_pid.y.fpDes = StartY + 0.5f * A_up_y * t4 * t4 + Vmax_y * (t_run - t1 - t2 - t4);
		p_nav->auto_path.basic_velt.fpVy = Vmax_y;
	}
	else if (t_run < t1 + t2 + t4 + t5 + t6)
	{
		p_nav->auto_path.pos_pid.y.fpDes = StartY + 0.5f * A_up_y * t4 * t4 + Vmax_y * t5 + Vmax_y * (t_run - t1 - t2 - t4 - t5) - 0.5f * A_down_y * (t_run - t1 - t2 - t4 - t5) * (t_run - t1 - t2 - t4 - t5);
		p_nav->auto_path.basic_velt.fpVy = Vmax_y - A_down_y * (t_run - t1 - t2 - t4 - t5);
	}
	else
	{
		p_nav->auto_path.pos_pid.y.fpDes = StartY + DELTA_Y;
		p_nav->auto_path.basic_velt.fpVy = 0;
	}

	if (t_run > (t1 + t2 + t4 + t5 + t6 + 0.1f) && t_run > (t1 + t2 + t3 + 0.1f))
	{
		p_nav->auto_path.run_time = 0;
		p_nav->state = NAV_HIT;

		p_nav->auto_path.pos_pid.x.fpSumE = 0;
		p_nav->auto_path.pos_pid.y.fpSumE = 0;
		p_nav->auto_path.pos_pid.w.fpSumE = 0;
	}
}

static void path_curve_1(cNav *p_nav)
{
	static fp32 A_up_x, A_up_y;
	static fp32 A_down_x, A_down_y;
//	static fp32 R;
	static fp32 Vmax_x;
	static fp32 Vmax_y;
	static fp32 Vw;
	static fp32 X1, X2, X3;
	static fp32 Y1, Y2, Y3;
//	static fp32 END_Q;

	if (flag_record)
	{
		A_up_x = 3000;
		A_down_x = 3000;
		A_up_y = 3500;
		A_down_y = 3000;

		Vmax_x = 3000;
		Vmax_y = 3500;

		StartX = p_nav->auto_path.pos_pid.x.fpFB;
		StartY = p_nav->auto_path.pos_pid.y.fpFB;
		StartQ = p_nav->auto_path.pos_pid.w.fpFB;

		turn_Q(point_end.q_end, StartQ);
		
		DELTA_X = point_end.x_end - StartX;
		DELTA_Y = point_end.y_end - StartY;
		DELTA_Q = point_end.q_end - StartQ;

		if (fabs(DELTA_X) <= fabs((Vmax_x * Vmax_x / (2 * A_up_x) + Vmax_x * Vmax_x / (2 * A_down_x))))
		{
			X1 = DELTA_X / (A_up_x + A_down_x) * A_down_x;
			X2 = 0;
			X3 = DELTA_X / (A_up_x + A_down_x) * A_up_x;
			t1 = sqrt(2 * X1 / A_up_x);
			t2 = 0;
			t3 = sqrt(2 * X3 / A_down_x);
			Vmax_x = A_up_x * t1;
		}
		else
		{
			t1 = Vmax_x / A_up_x;
			X1 = Vmax_x * Vmax_x / (2 * A_up_x);
			t3 = Vmax_x / A_down_x;
			X3 = Vmax_x * Vmax_x / (2 * A_down_x);
			X2 = DELTA_X - X1 - X3;
			t2 = X2 / Vmax_x;
		}

		if (fabs(DELTA_Y) <= fabs((Vmax_y * Vmax_y / (2 * A_up_y) + Vmax_y * Vmax_y / (2 * A_down_y))))
		{
			Y1 = DELTA_Y / (A_up_y + A_down_y) * A_down_y;
			Y2 = 0;
			Y3 = DELTA_Y / (A_up_y + A_down_y) * A_up_y;
			t4 = sqrt(2 * Y1 / A_up_y);
			t5 = 0;
			t6 = sqrt(2 * Y3 / A_down_y);
			Vmax_y = A_up_y * t4;
		}
		else
		{
			t4 = Vmax_y / A_up_y;
			Y1 = Vmax_y * Vmax_y / (2 * A_up_y);
			t6 = Vmax_y / A_down_y;
			Y3 = Vmax_y * Vmax_y / (2 * A_down_y);
			Y2 = DELTA_Y - Y1 - Y3;
			t5 = Y2 / Vmax_y;
		}

		Vw = DELTA_Q / (t1 + t2 + t4 + t5 + t6);

		p_nav->auto_path.pos_pid.x.fpSumE = 0;
		p_nav->auto_path.pos_pid.y.fpSumE = 0;
		p_nav->auto_path.pos_pid.w.fpSumE = 0;
		flag_record = 0;
	}

	t_run = Ts * p_nav->auto_path.run_time;

	if (t_run < t1 + t2 + t4 + t5 + t6)
	{
		p_nav->auto_path.pos_pid.w.fpDes = StartQ + Vw * t_run;
		p_nav->auto_path.basic_velt.fpW = Vw;
	}
	else
	{
		p_nav->auto_path.pos_pid.w.fpDes = 0;
		p_nav->auto_path.basic_velt.fpW = 0;
	}

	if (t_run < t1)
	{
		p_nav->auto_path.pos_pid.x.fpDes = StartX + 0.5f * A_up_x * t_run * t_run;
		p_nav->auto_path.basic_velt.fpVx = A_up_x * t_run;
	}
	else if (t_run < t1 + t2)
	{
		p_nav->auto_path.pos_pid.x.fpDes = StartX + 0.5f * A_up_x * t1 * t1 + Vmax_x * (t_run - t1);
		p_nav->auto_path.basic_velt.fpVx = Vmax_x;
	}
	else if (t_run < t1 + t2 + t3)
	{
		p_nav->auto_path.pos_pid.x.fpDes = StartX + 0.5f * A_up_x * t1 * t1 + Vmax_x * t2 + Vmax_x * (t_run - t1 - t2) - 0.5f * A_down_x * (t_run - t1 - t2) * (t_run - t1 - t2);
		p_nav->auto_path.basic_velt.fpVx = Vmax_x - A_down_x * (t_run - t1 - t2);
	}
	else
	{
		p_nav->auto_path.pos_pid.x.fpDes = StartX + DELTA_X;
		p_nav->auto_path.basic_velt.fpVx = 0;
	}

	if (t_run < t1 + t2)
	{
		p_nav->auto_path.pos_pid.y.fpDes = StartY;
		p_nav->auto_path.basic_velt.fpVy = 0;
	}
	else if (t_run < t1 + t2 + t4)
	{
		p_nav->auto_path.pos_pid.y.fpDes = StartY + 0.5f * A_up_y * (t_run - t1 - t2) * (t_run - t1 - t2);
		p_nav->auto_path.basic_velt.fpVy = A_up_y * (t_run - t1 - t2);
	}
	else if (t_run < t1 + t2 + t4 + t5)
	{
		p_nav->auto_path.pos_pid.y.fpDes = StartY + 0.5f * A_up_y * t4 * t4 + Vmax_y * (t_run - t1 - t2 - t4);
		p_nav->auto_path.basic_velt.fpVy = Vmax_y;
	}
	else if (t_run < t1 + t2 + t4 + t5 + t6)
	{
		p_nav->auto_path.pos_pid.y.fpDes = StartY + 0.5f * A_up_y * t4 * t4 + Vmax_y * t5 + Vmax_y * (t_run - t1 - t2 - t4 - t5) - 0.5f * A_down_y * (t_run - t1 - t2 - t4 - t5) * (t_run - t1 - t2 - t4 - t5);
		p_nav->auto_path.basic_velt.fpVy = Vmax_y - A_down_y * (t_run - t1 - t2 - t4 - t5);
	}
	else
	{
		p_nav->auto_path.pos_pid.y.fpDes = StartY + DELTA_Y;
		p_nav->auto_path.basic_velt.fpVy = 0;
	}

	if (t_run > (t1 + t2 + t4 + t5 + t6 + 0.1f) && t_run > (t1 + t2 + t3 + 0.1f))
	{
		p_nav->auto_path.run_time = 0;
		p_nav->state = NAV_HIT;

		p_nav->auto_path.pos_pid.x.fpSumE = 0;
		p_nav->auto_path.pos_pid.y.fpSumE = 0;
		p_nav->auto_path.pos_pid.w.fpSumE = 0;
	}
}

float DELTA_S;
//直线y+方向测试
static void path_openloop(cNav *p_nav)
{
//	static fp32 S;
	static fp32 S1, S2, S3;
	static fp32 A_up;
	static fp32 A_down;
	// fp32 vision_angle;
	//  static fp32 S_x,S_y;
//	static fp32 R;
	static fp32 Vmax;
	// VISION_POT.fpPosQ = WAIT_angle;
	if (flag_record) //初始化加速度，运动时间，位移etc
	{
		A_up = 3000;   //-4500;
		A_down = 1000; //-4500;
		Vmax = 600;
		DELTA_S = 300;

		StartX = p_nav->auto_path.pos_pid.x.fpFB;
		StartY = p_nav->auto_path.pos_pid.y.fpFB;
		StartQ = p_nav->auto_path.pos_pid.w.fpFB;

		S1 = Vmax * Vmax / (2 * A_up);
		S3 = Vmax * Vmax / (2 * A_down);
		S2 = DELTA_S - S1 - S3;
		t1 = Vmax / A_up;
		t3 = Vmax / A_down;
		t2 = S2 / Vmax;

		p_nav->auto_path.pos_pid.x.fpSumE = 0;
		p_nav->auto_path.pos_pid.y.fpSumE = 0;
		p_nav->auto_path.pos_pid.w.fpSumE = 0;
		flag_record = 0;
	}

	t_run = Ts * p_nav->auto_path.run_time;
	if (t_run < t1)
	{
		p_nav->auto_path.basic_velt.fpVx = A_up * t_run * cosf(DQ);
		p_nav->auto_path.basic_velt.fpVy = A_up * t_run * sinf(DQ);
		p_nav->auto_path.basic_velt.fpW = 0;
		// p_nav->auto_path.acceleration = A_up;
	}
	else if (t_run < t1 + t2)
	{
		p_nav->auto_path.basic_velt.fpVx = Vmax * cosf(DQ);
		p_nav->auto_path.basic_velt.fpVy = Vmax * sinf(DQ);
		// p_nav->auto_path.acceleration    = 0;
		p_nav->auto_path.basic_velt.fpW = 0;
	}
	else if (t_run < t1 + t2 + t3)
	{
		p_nav->auto_path.basic_velt.fpVx = (Vmax - A_down * (t_run - t1 - t2)) * cosf(DQ);
		p_nav->auto_path.basic_velt.fpVy = (Vmax - A_down * (t_run - t1 - t2)) * sinf(DQ);
		// p_nav->auto_path.acceleration    = 0;
		p_nav->auto_path.basic_velt.fpW = 0;
	}
	else if (t_run < t1 + t2 + t3 + 0.1f)
	{
		p_nav->auto_path.basic_velt.fpVx = 0;
		p_nav->auto_path.basic_velt.fpVy = 0;
		// p_nav->auto_path.acceleration    = 0;
		p_nav->auto_path.basic_velt.fpW = 0;
	}
	else
	{
		p_nav->auto_path.run_time = 0;
		p_nav->state = NAV_NEW_MANUAL;

		p_nav->auto_path.pos_pid.x.fpSumE = 0;
		p_nav->auto_path.pos_pid.y.fpSumE = 0;
		p_nav->auto_path.pos_pid.w.fpSumE = 0;
	}
	// p_nav->auto_path.basic_velt.fpW *= 0.5f;
}
static void my_test_path(cNav *p_nav)
{
	static fp32 R;
	static fp32 S1, S2, S3, S4, S5;
	static fp32 A_up;
	static fp32 A_down;
	static fp32 Vmax;
	static fp32 V_move;
	static fp32 S_move;
	static fp32 W_move;
	static fp32 W_r;
   fp32 dt;

	if (flag_record) //初始化加速度，运动时间，位移etc
	{
		//取块3000,1000,2000
		//回交接点3000,3000,2000
		//跑路径3000,3000,3000
		A_up = vanda.A_up;	   // 3000;
		A_down = vanda.A_down; // 3000;
		Vmax = vanda.Vmax;	   // 3000;

		StartX = p_nav->auto_path.pos_pid.x.fpFB;
		StartY = p_nav->auto_path.pos_pid.y.fpFB;
		StartQ = p_nav->auto_path.pos_pid.w.fpFB;

		//类似转向的控制
//		turn_Q(point_end.q_end, StartQ);

//		DELTA_X = point_end.x_end - StartX;
//		DELTA_Y = point_end.y_end - StartY;
//		DELTA_Q = point_end.q_end - StartQ;
		DELTA_X = 1000.0;
		DELTA_Y = 1000.0;
		DELTA_Q = 90.0;
    R = 1000.0;
		//可能要调整绝对式Q大小
		if (fabs(DELTA_Q) < 5)
		{
			StartQ = point_end.q_end;
			DELTA_Q = 0;
		}

			S1 = (Vmax * Vmax)/(A_up * 2);
			S2 = DELTA_X - S1;
		  S3 = PI * R / 2;
			S5 = (Vmax * Vmax)/(A_down * 2);
		  S4 = DELTA_Y - S5;
		
			t1 = Vmax / A_up;
			t2 = S2 / Vmax;
			t3 = S3 / Vmax;
		  t4 = S4 / Vmax;
		  t5 = Vmax / A_down;
		  
		
		W_move = DELTA_Q / t3;
		if (fabs(W_move) >= 90)
		{
			W_move = 0;
			DELTA_Q = 0;
		}

		p_nav->auto_path.pos_pid.x.fpSumE = 0;
		p_nav->auto_path.pos_pid.y.fpSumE = 0;
		p_nav->auto_path.pos_pid.w.fpSumE = 0;
		flag_record = 0;
		p_nav->auto_path.run_time = 0;
	}

	t_run = Ts * p_nav->auto_path.run_time;
	if (t_run < t1)
	{
		S_move = 0.5f * A_up * t_run * t_run;
		V_move = A_up * t_run;
		p_nav->auto_path.pos_pid.x.fpDes = StartX + S_move;
		p_nav->auto_path.pos_pid.y.fpDes = StartY ;
		p_nav->auto_path.basic_velt.fpVx = V_move ;
		p_nav->auto_path.basic_velt.fpVy = 0;
		p_nav->auto_path.pos_pid.w.fpDes = StartQ;
		p_nav->auto_path.basic_velt.fpW = 0;
	}
	else if (t_run < t1 + t2)
	{
		S_move = S1 + (t_run - t1)*Vmax;
		V_move = Vmax;
		p_nav->auto_path.pos_pid.x.fpDes = StartX + S_move;
		p_nav->auto_path.pos_pid.y.fpDes = StartY ;
		p_nav->auto_path.basic_velt.fpVx = V_move ;
		p_nav->auto_path.basic_velt.fpVy = 0;
		p_nav->auto_path.pos_pid.w.fpDes = StartQ;
		p_nav->auto_path.basic_velt.fpW = 0;
	}
	else if (t_run < t1 + t2 + t3)
	{
		S_move = S1 + S2 + (t_run - t1 - t2)*Vmax;
		V_move = Vmax ;
		W_r = PI/2 *((t_run - t1 - t2)) / t3;
		p_nav->auto_path.pos_pid.x.fpDes = StartX + S1 + S2 + R * sinf(W_r);
		p_nav->auto_path.pos_pid.y.fpDes = StartY + R * (1 - cosf(W_r));
		p_nav->auto_path.basic_velt.fpVx = V_move * cosf(W_r);
		p_nav->auto_path.basic_velt.fpVy = V_move * sinf(W_r);
		p_nav->auto_path.pos_pid.w.fpDes = StartQ + W_move * (t_run - t1 - t2);
		p_nav->auto_path.basic_velt.fpW = W_move;
	}
	else if (t_run < t1 + t2 + t3 + t4)
	{
		S_move = R + (t_run - t1 - t2 - t3)*Vmax;
		V_move = Vmax ;
		p_nav->auto_path.pos_pid.x.fpDes = StartX + S1 + S2 + R;
		p_nav->auto_path.pos_pid.y.fpDes = StartY + S_move ;
		p_nav->auto_path.basic_velt.fpVx = 0;
		p_nav->auto_path.basic_velt.fpVy = V_move;
		p_nav->auto_path.pos_pid.w.fpDes = StartQ + DELTA_Q;
		p_nav->auto_path.basic_velt.fpW = 0;
	}
	else if (t_run < t1 + t2 + t3 + t4 + t5)
	{
		dt = (t_run - t1 - t2 - t3 - t4);
		S_move = R + S4 + Vmax*dt - 0.5f *dt*dt*A_down;
		V_move = Vmax - A_down * dt;
		p_nav->auto_path.pos_pid.x.fpDes = StartX + S1 + S2 + R;
		p_nav->auto_path.pos_pid.y.fpDes = StartY + S_move ;
		p_nav->auto_path.basic_velt.fpVx = 0;
		p_nav->auto_path.basic_velt.fpVy = V_move;
		p_nav->auto_path.pos_pid.w.fpDes = StartQ + DELTA_Q;
		p_nav->auto_path.basic_velt.fpW = 0;
	}
	else if (t_run < t1 + t2 + t3 + t4 + t5 + 0.1f)
	{
		p_nav->auto_path.pos_pid.x.fpDes = StartX + S1 + S2 + R;
		p_nav->auto_path.pos_pid.y.fpDes = StartY + S4 + S5 + R ;
		p_nav->auto_path.basic_velt.fpVx = 0;
		p_nav->auto_path.basic_velt.fpVy = 0;
		p_nav->auto_path.pos_pid.w.fpDes = StartQ + DELTA_Q;
		p_nav->auto_path.basic_velt.fpW = 0;
	}
	else
	{
		p_nav->auto_path.run_time = 0;
		p_nav->state = NAV_STOP;

		p_nav->auto_path.pos_pid.x.fpSumE = 0;
		p_nav->auto_path.pos_pid.y.fpSumE = 0;
		p_nav->auto_path.pos_pid.w.fpSumE = 0;
	}	

	
}

void chassis_path_choose(cNav *p_nav, ST_Trajectory *track)
{
	switch (p_nav->auto_path.number)
	{
	case 0:
		// none
		break;
	case 1:
		path_straight(p_nav);
		break;
	case 2:
		path_curve(p_nav);
		break;
	case 3:
		path_curve_1(p_nav);
	  break;
	case 10:
		path_x_test(p_nav);
		break;
	case 11:
		path_y_test(p_nav);
		break;
	case 20:
		path_openloop(p_nav);
		break;
	case 22:
		my_test_path(p_nav);
		break;
	default:
		break;
	}
}

//斜坡函数
bool ramp_signal(float &output,float des ,float dt,float total_t,uint8_t &flag, float &step)
{
	if(flag == 1)
	{
	  step = (des - output) / ( total_t / dt );
	  //太小
		if(fabs(output - des) <= 0.0001)
	   {
		   flag = 3;
		   return 1;
	   }

	  flag = 2;
  }
	
	if(flag == 1 || flag == 2)
	{
	output += step;	
	if(fabs(output - des) <= fabs(step))
	{
		flag = 3;
		return 1;
	}
	}
	if(flag == 3)
	{
		return 1;	
	}
	
	return 0;
	
}

//sin函数  输入： 时间t  输出：y = H + A * sin(2*pi/T * (t + phi))  一般phi = 0;
//T为周期，total_t为总时间
bool sin_signal(float &output,float A, float H, float T,float dt,float total_t ,float phi ,float &real_t)
{
	if(real_t < total_t)
	{
	  output = H + A * sin( 2 * PI / T * (real_t + phi));
		real_t += dt;
	}
	else if(real_t >= total_t)
	{
		return 1;
	}
	return 0;
}




