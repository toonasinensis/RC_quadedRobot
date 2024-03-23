#include "locate_algorithm.h"

/****************定位使用******************/
fp32 fpPosXOffset = 0; // X方向纠偏量，勿动
fp32 fpPosYOffset = 0; // Y方向纠偏量，勿动
fp32 fpQOffset = 0;    //角度Q纠偏量，勿动

fp32 fpStartX = 0; // 5557.5f;//319 底盘半宽+导轮
fp32 fpStartY = 0; // 545.0f;//361

uint32_t cnt_Gyro;
uint32_t fps_Gyro;
fp32 cha_x; //插值因子

float hide_y;

/***************测速使用*******************/
static fp32 afpRobot_Q[4] = {0};       //存储机器人姿态角
static uint32_t auiTIM2_Time[4] = {0}; //存储TIM2计数值
fp32 afpRobot_PosX[4] = {0};           //存储机器人X位姿
fp32 afpRobot_PosY[4] = {0};           //存储机器人Y位姿
void Push_Q_In(fp32 fpQ);              //这些数组储存了最新记录的4个相关数据
void Push_Time_In(uint32_t uiT);
void Push_RobotPosX_In(fp32 fpX);
void Push_RobotPosY_In(fp32 fpY);

void Calibrate_Robot_Degree(ST_ROBOT *pstRobot, ST_GYRO *pstGyro);

//随动轮结构体声明
ST_FOLLOWER_WHEEL stFollowerWheel =
    {
        0,
        0,
        0,
        0,
        {0, 0, 0, FW_Rob_Len, FW_rob_Alpha},
        {0},
        {0}};

//陀螺仪结构体声明
ST_GYRO stGyro =
    {
        K_CLOCK,     //顺时针
        K_ANTICLOCK, //逆时针
        0,
        0};

dt35_t dt35_save =
    {
        0.05f, 0.61f, 0.00f,
        2869.0f, 3094.5f, 1478.0f, 1491.0f,
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0};

dt35_t dt35_now =
    {
        0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0};

/*******************************************************************************************
函数名称：RobotLocation()
函数功能：由双随动轮和陀螺仪数据得到机器人的坐标和姿态角（单位：mm，0.1°）
输入：	  1.pstRobot 指向机器人总结构体的指针
          2.pstFW	 指向随动轮总结构体的指针
          3.pstGyro  指向陀螺总结构体的指针
输出：	  1.机器人中心位姿，包括坐标和姿态角
          2.随动轮中心位姿	包括坐标
备注：    1.此函数中的运算都是在弧度为单位的情况下进行的
          2.ALPHA_A为A随动轮逆时针旋转时线速度方向与机器人局部坐标系y正方向的夹角
              3.ALPHA_B为B随动轮逆时针旋转时线速度方向与机器人局部坐标系y正方向的夹角
          4.以上两个角度就是从y轴逆旋为正
*******************************************************************************************/
ST_LPF Fpx = {0, 0, 0, 20, 0.002};
ST_LPF Fpy = {0, 0, 0, 20, 0.002};
fp32 temp_x, temp_y;
void RobotLocation(ST_ROBOT *pstRobot, ST_FOLLOWER_WHEEL *pstFW, ST_GYRO *pstGyro)
{
    fp32 ALPHA_B;
    fp32 ALPHA_A;
    fp32 Sin_B_A;
    fp32 fpDeltaA, fpDeltaB;  //随动轮走过的距离
    fp32 Convert_Array[2][2]; //距离转换矩阵，实际是3*3的，但是我们只用2*2
    fp32 fpQ;                 //机器人姿态角临时变量(单位：弧度)

	  if(nav.state==NAV_NEW_MANUAL||NAV_AUTO_PATH)
		{
			pstGyro->fpClock=K_CLOCK;
			pstGyro->fpAntiClock=K_ANTICLOCK;
		}
		else{
     	pstGyro->fpClock=K_CLOCK;
			pstGyro->fpAntiClock=K_ANTICLOCK;
		}
	  
    /*******************获取角度*********************/
    Calibrate_Robot_Degree(pstRobot, pstGyro);                    //给pstRobot->stPot.fpPosQ1赋值,单位0.1度
    pstRobot->stPot.fpPosQ = pstRobot->stPot.fpPosQ1 + fpQOffset; //单位0.1度
    fpQ = ConvertAngle(pstRobot->stPot.fpPosQ * RADIAN_10);       //单位:弧度

    /**************获取随动轮方向位移****************/
    pstFW->siCoderACur = degreeA;
    pstFW->siCoderBCur = degreeB;

    if (my_intabs(pstFW->siCoderACur - pstFW->siCoderAPre) > 1000 ||
        my_intabs(pstFW->siCoderBCur - pstFW->siCoderBPre) > 1000) // 2ms 10cm不可能
    {
        pstFW->siCoderAPre = pstFW->siCoderACur;
        pstFW->siCoderBPre = pstFW->siCoderBCur;
        return;
    }

    /*计算随动轮走过的距离*/
    if (pstFW->siCoderACur >= pstFW->siCoderAPre) // A轮正转，正转编码器值变小
    {
        fpDeltaA = (pstFW->siCoderACur - pstFW->siCoderAPre) * FW_Len_A_Inc;
        ALPHA_A = ALPHA_A_Inc;
    }
    else // A轮反转
    {
        fpDeltaA = (pstFW->siCoderACur - pstFW->siCoderAPre) * FW_Len_A_Dec;
        ALPHA_A = ALPHA_A_Dec;
    }
		
    if (pstFW->siCoderBCur >= pstFW->siCoderBPre) // B轮正转
    {
        fpDeltaB = (pstFW->siCoderBCur - pstFW->siCoderBPre) * FW_Len_B_Inc;
        ALPHA_B = ALPHA_B_Inc;
    }
    else // B轮反转
    {
        fpDeltaB = (pstFW->siCoderBCur - pstFW->siCoderBPre) * FW_Len_B_Dec;
        ALPHA_B = ALPHA_B_Dec;
    }

    /**************解算随动轮中心坐标****************/
    Sin_B_A = sinf(ALPHA_B - ALPHA_A); //转换矩阵分母系数sin(B-A)

    Convert_Array[0][0] = cosf(ALPHA_B + fpQ) / Sin_B_A;
    Convert_Array[0][1] = cosf(ALPHA_A + fpQ) / Sin_B_A;
    Convert_Array[1][0] = sinf(ALPHA_B + fpQ) / Sin_B_A;
    Convert_Array[1][1] = sinf(ALPHA_A + fpQ) / Sin_B_A;

    pstFW->stPot.fpPosX += Convert_Array[0][0] * fpDeltaA - Convert_Array[0][1] * fpDeltaB;
    pstFW->stPot.fpPosY += Convert_Array[1][0] * fpDeltaA - Convert_Array[1][1] * fpDeltaB;

    /**************解算机器人中心坐标****************/
    /*任何位置都适用，其中要注意FW_rob_Alpha为机器人中心指向随动轮中心的矢量(同样要求从y轴开始逆时针旋转为正)，该角度范围为[0，2*pi)*/
    pstRobot->stPot.fpPosX = fpStartX + pstFW->stPot.fpPosX - (-sinf(-FW_rob_Alpha) + sinf(-FW_rob_Alpha - fpQ)) * pstFW->stVectorFWCen_RobCen.fpLength;
    pstRobot->stPot.fpPosY = fpStartY + pstFW->stPot.fpPosY + (cosf(-FW_rob_Alpha) - cosf(-FW_rob_Alpha - fpQ)) * pstFW->stVectorFWCen_RobCen.fpLength;

    pstRobot->stPot.fpPosX = pstRobot->stPot.fpPosX + fpPosXOffset;
    pstRobot->stPot.fpPosY = pstRobot->stPot.fpPosY + fpPosYOffset;

//    temp_x = pstRobot->stPot.fpPosY;
//    temp_y = -pstRobot->stPot.fpPosX;

//    pstRobot->stPot.fpPosX = temp_x;
//    pstRobot->stPot.fpPosY = temp_y;

    /*随动轮编码器数据保存*/
    pstFW->siCoderAPre = pstFW->siCoderACur;
    pstFW->siCoderBPre = pstFW->siCoderBCur;
}

/*******************************************************************************************
函数名称：CalibrateRobotVelt()
函数功能：校准机器人的速度，由求得的坐标增量计算机器人在全局坐标系下的速度（单位mm/s）
输入：	  1.pstRobot 指向机器人总结构体的指针
输出：	  1.机器人在全局坐标系下的速度
备注：    1.函数中角速度的单位是(0.1度/秒)
*******************************************************************************************/
ST_LPF Fvx = {0, 0, 0, 200, 0.002};
ST_LPF Fvy = {0, 0, 0, 200, 0.002};
ST_LPF Fw = {0, 0, 0, 20, 0.002};
void CalibrateRobotVelt(ST_ROBOT *pstRobot)
{
    /*存储机器人坐标、姿态角、计数器计数值的数组更新*/
    Push_Q_In(pstRobot->stPot.fpPosQ);
    Push_Time_In(TIM2->CNT);
    Push_RobotPosX_In(pstRobot->stPot.fpPosX);
    Push_RobotPosY_In(pstRobot->stPot.fpPosY);

    /*滤波*/
    Fvx.in = (afpRobot_PosX[0] - afpRobot_PosX[3]) / ((auiTIM2_Time[0] - auiTIM2_Time[3]) * TIM12_BASE_TIME); // x方向速度
    Fvy.in = (afpRobot_PosY[0] - afpRobot_PosY[3]) / ((auiTIM2_Time[0] - auiTIM2_Time[3]) * TIM12_BASE_TIME); // y方向速度
    Fw.in = (afpRobot_Q[0] - afpRobot_Q[3]) / ((auiTIM2_Time[0] - auiTIM2_Time[3]) * TIM12_BASE_TIME)/10.0f;        //角速度，方向从y轴逆转为正

    LpFilter(&Fvx);
    LpFilter(&Fvy);
    LpFilter(&Fw);

    pstRobot->stVelt.fpVx = Fvx.out;
    pstRobot->stVelt.fpVy = Fvy.out;
    pstRobot->stVelt.fpW = Fw.out;
}

/*******************************************************************************************
函数名称：Calibrate_Robot_Degree()
函数功能：由陀螺数据得到机器人的姿态角
输入：	  1.pstRobot 指向机器人总结构体的指针
          2.pstGyro  指向陀螺总结构体的指针
输出：	  1.机器人姿态角
          2.陀螺结构体数据更新
备注：    1.函数中涉及的角度单位都是0.1度(以后的函数中Degree代表角度数，Angle则代表弧度数)
          2.逆时针和顺时针旋转均有标定系数在陀螺仪的结构体中声明
          3.机器人姿态角是指机器人坐标系y轴与全局坐标系Y轴的夹角
          4.随动轮中心姿态角是指机器人中心至随动轮中心的向量与全局坐标系Y轴的夹角
*******************************************************************************************/

static fp32 get_gyro_value(void)
{
    static bool change_flag = 0;
    static int32_t gyro_value[2] = {0}; //该数组为最后两次的陀螺仪数值，去掉所有重复的数值
    int32_t gyro_value_now;

//    gyro_value_now = TestAngle2;

    if (gyro_value_now != gyro_value[1]) //如果陀螺仪数据发生了变化，则更新该数组
    {
        gyro_value[0] = gyro_value[1];
        gyro_value[1] = gyro_value_now;
        change_flag = 1;
    }
    else
    {
        change_flag = 0;
    }

    if (change_flag == 1)
    {
        return (fp32)(gyro_value[0] + gyro_value[1]) / 2;
    }
    else
    {
        return (fp32)(gyro_value[1]);
    }
}

// fp32 raw_q;
// uint32_t gyro_error_cnt = 0;

void Calibrate_Robot_Degree(ST_ROBOT *pstRobot, ST_GYRO *pstGyro)
{
//    //	raw_q = (fp32)GYRO_Q;
//    // pstGyro->fpQ_Cur = (fp32)GYRO_Q;
//    pstGyro->fpQ_Cur = get_gyro_value();
////    pstGyro->fpQ_Cur = rc19_sec.Gyro_sec;
////	  if(pstGyro->fpQ_Cur-pstGyro->fpQ_Pre>1000)
////			pstGyro->fpQ_Pre = pstGyro->fpQ_Pre + 3600;
////		else if(pstGyro->fpQ_Cur-pstGyro->fpQ_Pre<-1000)
////			pstGyro->fpQ_Pre = pstGyro->fpQ_Pre - 3600;
	
#if GambalGyro //云台mpu6050返回的
	
    if (fabs(pstGyro->fpQ_Cur - pstGyro->fpQ_Pre) > 100) // 2ms转10度，不可能
    {
        pstRobot->stPot.fpPosQ1 += 0;
    }
    else if ((pstGyro->fpQ_Cur - pstGyro->fpQ_Pre >= 0)||(pstGyro->fpQ_Cur - pstGyro->fpQ_Pre <= -1800.0)) //逆时针
    {
        pstRobot->stPot.fpPosQ1 += (pstGyro->fpQ_Cur - pstGyro->fpQ_Pre) * pstGyro->fpAntiClock;
    }
    else if((pstGyro->fpQ_Cur - pstGyro->fpQ_Pre <= 0)||(pstGyro->fpQ_Cur - pstGyro->fpQ_Pre >= 1800.0))//顺时针
    {
        pstRobot->stPot.fpPosQ1 += (pstGyro->fpQ_Cur - pstGyro->fpQ_Pre) * pstGyro->fpClock;
    }

    if (pstGyro->fpQ_Pre != pstGyro->fpQ_Cur)
    {
        fps_Gyro = 500 / cnt_Gyro;
        cnt_Gyro = 0;
    }
    else
        cnt_Gyro++;
    pstGyro->fpQ_Pre = pstGyro->fpQ_Cur; //陀螺角度数据更新
	
#endif
	
#if DigtalGyro //数字
               //	if(cha_x==0)
               //	{
               //		pstGyro->fpQ_Cur += (pstGyro->fpQ_Cur - pstGyro->fpQ_Pre)/2.0f;
               //		cha_x=1;
               //	}
               //	else if(cha_x==1)
               //	{
               //		cha_x=0;//不插值
               //	}
    pstGyro->fpQ_Cur = get_gyro_value();
    if (fabs(pstGyro->fpQ_Cur - pstGyro->fpQ_Pre) > 100) // 2ms转10度，不可能
    {
        pstRobot->stPot.fpPosQ1 += 0;
        // gyro_error_cnt++;
    }
    else if (pstGyro->fpQ_Cur - pstGyro->fpQ_Pre >= 0) //逆时针
    {
        pstRobot->stPot.fpPosQ1 += (pstGyro->fpQ_Cur - pstGyro->fpQ_Pre) * pstGyro->fpAntiClock;
    }
    else //顺时针
    {
        pstRobot->stPot.fpPosQ1 += (pstGyro->fpQ_Cur - pstGyro->fpQ_Pre) * pstGyro->fpClock;
    }

    if (pstGyro->fpQ_Pre != pstGyro->fpQ_Cur)
    {
        fps_Gyro = 500 / cnt_Gyro;
        cnt_Gyro = 0;
    }
    else
        cnt_Gyro++;
    pstGyro->fpQ_Pre = pstGyro->fpQ_Cur; //陀螺角度数据更新

#endif

#if AnologGyro                                //模拟
    pstGyro->fpQ_Cur = get_gyro_value();
    if (pstGyro->fpQ_Cur >= pstGyro->fpQ_Pre) //逆时针
    {
        pstRobot->stPot.fpPosQ1 += (pstGyro->fpQ_Cur - pstGyro->fpQ_Pre) * pstGyro->fpAntiClock;
    }
    else //顺时针
    {
        pstRobot->stPot.fpPosQ1 += (pstGyro->fpQ_Cur - pstGyro->fpQ_Pre) * pstGyro->fpClock;
    }
    pstGyro->fpQ_Pre = pstGyro->fpQ_Cur; //陀螺角度数据更新
#endif
}

/*******************************************************************************************
函数名称：Push_Q_In()
函数功能：将新的角度值存入数组中
输入：	  fpQ  待存入的角度值
输出：	  无
备注：
*******************************************************************************************/
void Push_Q_In(fp32 fpQ)
{
    uint8_t i;
    for (i = 3; i >= 1; i--)
    {
        afpRobot_Q[i] = afpRobot_Q[i - 1];
    }
    afpRobot_Q[0] = fpQ;
}
/*******************************************************************************************
函数名称：Push_Time_In()
函数功能：将新的计数值存入数组中
输入：	  uiT  待存入的计数值
输出：	  无
备注：
*******************************************************************************************/
void Push_Time_In(uint32_t uiT)
{
    uint8_t i;
    for (i = 3; i >= 1; i--)
    {
        auiTIM2_Time[i] = auiTIM2_Time[i - 1];
    }
    auiTIM2_Time[0] = uiT;
}
/*******************************************************************************************
函数名称：Push_RobotPosX_In()
函数功能：将机器人新的X位姿存入数组
输入：	  fpX  待存入的计数值
输出：	  无
备注：
*******************************************************************************************/
void Push_RobotPosX_In(fp32 fpX)
{
    uint8_t i;
    for (i = 3; i >= 1; i--)
    {
        afpRobot_PosX[i] = afpRobot_PosX[i - 1];
    }
    afpRobot_PosX[0] = fpX;
}
/*******************************************************************************************
函数名称：Push_RobotPosY_In()
函数功能：将机器人新的Y位姿存入数组
输入：	  fpX  待存入的计数值
输出：	  无
备注：
*******************************************************************************************/
void Push_RobotPosY_In(fp32 fpY)
{
    uint8_t i;
    for (i = 3; i >= 1; i--)
    {
        afpRobot_PosY[i] = afpRobot_PosY[i - 1];
    }
    afpRobot_PosY[0] = fpY;
}

//四个DT35，在rightdown轮对应的边沿x正方向为y1,y2；在up轮对应的边沿y正方向为x2,x1
fp32 q_save_fix_x1, q_save_fix_y1, q_now_fix_x1, q_now_fix_y1, q_save_fix_x2, q_save_fix_y2, q_now_fix_x2, q_now_fix_y2;
fp32 q_fix_x1 = 0.0f, q_fix_x2 = 0.0f;
fp32 q_fix_y1 = 0.0f, q_fix_y2 = 0.0f;
float temp_x_dt35, temp_y_dt35;
float temp_x1_dt35, temp_y1_dt35;
float temp_x2_dt35, temp_y2_dt35;
float temp_y_hide_dt35, temp_y1_hide_dt35, temp_y2_hide_dt35;
float temp_x_hide_dt35, temp_x1_hide_dt35, temp_x2_hide_dt35;
bool flag_x_dt35, flag_y_dt35;

void DT35_relocation_new(ST_ROBOT *pstRobot, ST_FOLLOWER_WHEEL *pstFW, dt35_t *p_dt35_save, dt35_t *p_dt35_now)
{
    fp32 fpQ_now, fpQ_save;
    p_dt35_now->robot_q = pstRobot->stPot.fpPosQ;
    // x:4du
    fpQ_save = ConvertAngle(p_dt35_save->robot_q * RADIAN_10); //弧度
    fpQ_now = ConvertAngle(p_dt35_now->robot_q * RADIAN_10);   //弧度

    //四个dt35的修正角度，顺时针为正，逆时针为负
    q_save_fix_x1 = ConvertAngle((p_dt35_save->robot_q - q_fix_x1) * RADIAN_10);
    q_save_fix_y1 = ConvertAngle((p_dt35_save->robot_q - q_fix_y1) * RADIAN_10);
    q_now_fix_x1 = ConvertAngle((p_dt35_now->robot_q - q_fix_x1) * RADIAN_10);
    q_now_fix_y1 = ConvertAngle((p_dt35_now->robot_q - q_fix_y1) * RADIAN_10);
    q_save_fix_x2 = ConvertAngle((p_dt35_save->robot_q - q_fix_x2) * RADIAN_10);
    q_save_fix_y2 = ConvertAngle((p_dt35_save->robot_q - q_fix_y2) * RADIAN_10);
    q_now_fix_x2 = ConvertAngle((p_dt35_now->robot_q - q_fix_x2) * RADIAN_10);
    q_now_fix_y2 = ConvertAngle((p_dt35_now->robot_q - q_fix_y2) * RADIAN_10);

    // dt35到墙的距离，
    p_dt35_save->dt35_fix_x1 = fabs(K_DT35_X1 * p_dt35_save->dt35_x1 + B_DT35_X1);
    p_dt35_save->dt35_fix_x2 = fabs(K_DT35_X2 * p_dt35_save->dt35_x2 + B_DT35_X2);
    p_dt35_now->dt35_fix_x1 = fabs(K_DT35_X1 * p_dt35_now->dt35_x1 + B_DT35_X1);
    p_dt35_now->dt35_fix_x2 = fabs(K_DT35_X2 * p_dt35_now->dt35_x2 + B_DT35_X2);

    p_dt35_save->dt35_fix_y1 = fabs(K_DT35_Y1 * p_dt35_save->dt35_y1 + B_DT35_Y1);
    p_dt35_save->dt35_fix_y2 = fabs(K_DT35_Y2 * p_dt35_save->dt35_y2 + B_DT35_Y2);
    p_dt35_now->dt35_fix_y1 = fabs(K_DT35_Y1 * p_dt35_now->dt35_y1 + B_DT35_Y1);
    p_dt35_now->dt35_fix_y2 = fabs(K_DT35_Y2 * p_dt35_now->dt35_y2 + B_DT35_Y2);

    // dt35所在边的中心的x或y距离

    p_dt35_save->pro_x1 = p_dt35_save->dt35_fix_x1 * cosf(q_save_fix_x1) - DIS_X1_DT35_TO_C1 * sinf(q_save_fix_x1) + DIS_X1_DT35_TO_C2 * cosf(q_save_fix_x1);
    p_dt35_now->pro_x1 = p_dt35_now->dt35_fix_x1 * cosf(q_now_fix_x1) - DIS_X1_DT35_TO_C1 * sinf(q_now_fix_x1) + DIS_X1_DT35_TO_C2 * cosf(q_now_fix_x1);
    p_dt35_save->pro_x2 = p_dt35_save->dt35_fix_x2 * cosf(q_save_fix_x2) + DIS_X2_DT35_TO_C1 * sinf(q_save_fix_x2) + DIS_X2_DT35_TO_C2 * cosf(q_save_fix_x2);
    p_dt35_now->pro_x2 = p_dt35_now->dt35_fix_x2 * cosf(q_now_fix_x2) + DIS_X2_DT35_TO_C1 * sinf(q_now_fix_x2) + DIS_X2_DT35_TO_C2 * cosf(q_now_fix_x2);

    p_dt35_save->pro_y1 = p_dt35_save->dt35_fix_y1 * cosf(q_save_fix_y1) + DIS_Y1_DT35_TO_C1 * sinf(q_save_fix_y1) + DIS_Y1_DT35_TO_C2 * cosf(q_save_fix_y1);
    p_dt35_now->pro_y1 = p_dt35_now->dt35_fix_y1 * cosf(q_now_fix_y1) + DIS_Y1_DT35_TO_C1 * sinf(q_now_fix_y1) + DIS_Y1_DT35_TO_C2 * cosf(q_now_fix_y1);
    p_dt35_save->pro_y2 = p_dt35_save->dt35_fix_y2 * cosf(q_save_fix_y2) - DIS_Y2_DT35_TO_C1 * sinf(q_save_fix_y2) + DIS_Y2_DT35_TO_C2 * cosf(q_save_fix_y2);
    p_dt35_now->pro_y2 = p_dt35_now->dt35_fix_y2 * cosf(q_now_fix_y2) - DIS_Y2_DT35_TO_C1 * sinf(q_now_fix_y2) + DIS_Y2_DT35_TO_C2 * cosf(q_now_fix_y2);

    //	p_dt35_now->robot_x = p_dt35_save->robot_x + 0.5f * (p_dt35_now->pro_x1 - p_dt35_save->pro_x1 + p_dt35_now->pro_x2 -p_dt35_save->pro_x2);
    p_dt35_now->robot_x = p_dt35_save->robot_x - (p_dt35_now->pro_x1 - p_dt35_save->pro_x1 + p_dt35_now->pro_x2 - p_dt35_save->pro_x2) / 2;
    p_dt35_now->robot_x1 = p_dt35_save->robot_x - (p_dt35_now->pro_x1 - p_dt35_save->pro_x1);
    p_dt35_now->robot_x2 = p_dt35_save->robot_x - (p_dt35_now->pro_x2 - p_dt35_save->pro_x2);

    //    p_dt35_now->robot_x = p_dt35_save->robot_x + (p_dt35_now->pro_x1 - p_dt35_save->pro_x1);
    p_dt35_now->robot_y = p_dt35_save->robot_y + (p_dt35_now->pro_y2 - p_dt35_save->pro_y2 + p_dt35_now->pro_y1 - p_dt35_save->pro_y1) / 2;
    p_dt35_now->robot_y1 = p_dt35_save->robot_y + (p_dt35_now->pro_y1 - p_dt35_save->pro_y1);
    p_dt35_now->robot_y2 = p_dt35_save->robot_y + (p_dt35_now->pro_y2 - p_dt35_save->pro_y2);

    temp_x_dt35 = -p_dt35_now->robot_y;
    temp_y_dt35 = p_dt35_now->robot_x1;
    //		temp_x1_dt35 = p_dt35_now->robot_y1 * sinf(PI / 6.0) + p_dt35_now->robot_x1 * cosf(PI / 6.0);
    //    temp_y1_dt35 = p_dt35_now->robot_y1 * cosf(PI / 6.0) - p_dt35_now->robot_x1 * sinf(PI / 6.0);
    //		temp_x2_dt35 = p_dt35_now->robot_y2 * sinf(PI / 6.0) + p_dt35_now->robot_x2 * cosf(PI / 6.0);
    //    temp_y2_dt35 = p_dt35_now->robot_y2 * cosf(PI / 6.0) - p_dt35_now->robot_x2 * sinf(PI / 6.0);

    //		temp_x_hide_dt35 = (p_dt35_now->robot_y+HIDE_Y) * sinf(PI / 6.0) + p_dt35_now->robot_x * cosf(PI / 6.0);
    //		temp_x1_hide_dt35 = (p_dt35_now->robot_y1+HIDE_Y) * sinf(PI / 6.0) + p_dt35_now->robot_x * cosf(PI / 6.0);
    //		temp_x2_hide_dt35 = (p_dt35_now->robot_y2+HIDE_Y) * sinf(PI / 6.0) + p_dt35_now->robot_x * cosf(PI / 6.0);
    //		temp_y_hide_dt35 = (p_dt35_now->robot_y+HIDE_Y) * cosf(PI / 6.0) - p_dt35_now->robot_x * sinf(PI / 6.0);
    //		temp_y1_hide_dt35 = (p_dt35_now->robot_y1+HIDE_Y) * cosf(PI / 6.0) - p_dt35_now->robot_x * sinf(PI / 6.0);
    //		temp_y2_hide_dt35 = (p_dt35_now->robot_y2+HIDE_Y) * cosf(PI / 6.0) - p_dt35_now->robot_x * sinf(PI / 6.0);

    //    temp_x_dt35 = p_dt35_now->robot_x;
    //    temp_y_dt35 = p_dt35_now->robot_y;
    //    temp_x1_dt35 = p_dt35_now->robot_x1;
    //    temp_y1_dt35 = p_dt35_now->robot_y1;
    //    temp_x2_dt35 = p_dt35_now->robot_x2;
    //    temp_y2_dt35 = p_dt35_now->robot_y2;

    if ((nav.state == NAV_MANUAL ) && (keyboard_mode != ACTION))
    {
        pstFW->stPot.fpPosY = temp_y_dt35 - (cosf(-FW_rob_Alpha) - cosf(-FW_rob_Alpha - fpQ_now)) * pstFW->stVectorFWCen_RobCen.fpLength;
        pstFW->stPot.fpPosY1 = temp_y_dt35 - (cosf(-FW_rob_Alpha) - cosf(-FW_rob_Alpha - fpQ_now)) * pstFW->stVectorFWCen_RobCen.fpLength;
        pstFW->stPot.fpPosX = temp_x_dt35 + (-sinf(-FW_rob_Alpha) + sinf(-FW_rob_Alpha - fpQ_now)) * pstFW->stVectorFWCen_RobCen.fpLength;
        pstFW->stPot.fpPosX1 = temp_x_dt35 + (-sinf(-FW_rob_Alpha) + sinf(-FW_rob_Alpha - fpQ_now)) * pstFW->stVectorFWCen_RobCen.fpLength;
    }
		else if(nav.state ==NAV_DT35_RELOC|| nav.state == NAV_NEW_MANUAL)
		{
			  flag_x_dt35 = 1;
        flag_y_dt35 = 1;
			  if(fabs(p_dt35_now->robot_x1 - p_dt35_now->robot_x2) <= 20.0)
				{
					p_dt35_now->robot_x = p_dt35_now->robot_x;
				}
				else if(p_dt35_now->robot_x1<p_dt35_now->robot_x2)
				{
					p_dt35_now->robot_x = p_dt35_now->robot_x1;
				}
				else if(p_dt35_now->robot_x1>p_dt35_now->robot_x2)
				{
					p_dt35_now->robot_x = p_dt35_now->robot_x2;
				} 
				else
        {
            flag_x_dt35 = 0;
        }
       

        if (fabs(pstRobot->stPot.fpPosX) < 1000 && fabs(pstRobot->stPot.fpPosQ) / 10.0f < 50.0f)
        {
                p_dt35_now->robot_y = p_dt35_now->robot_y;  
        }
        else if (fabs(pstRobot->stPot.fpPosX) > 1000&&fabs(pstRobot->stPot.fpPosQ) / 10.0f < 50.0f)
        {
                p_dt35_now->robot_y = p_dt35_now->robot_y + HIDE_Y;
        }
				else
				{
					flag_y_dt35 = 0;
				}

	//        if (flag_x_dt35 && flag_y_dt35)
	//        {
	//					  temp_x_dt35 = -p_dt35_now->robot_y;
	//            temp_y_dt35 = p_dt35_now->robot_x;
	//            pstFW->stPot.fpPosY = temp_y_dt35 - (cosf(-FW_rob_Alpha) - cosf(-FW_rob_Alpha - fpQ_now)) * pstFW->stVectorFWCen_RobCen.fpLength;
	//            pstFW->stPot.fpPosY1 = temp_y_dt35 - (cosf(-FW_rob_Alpha) - cosf(-FW_rob_Alpha - fpQ_now)) * pstFW->stVectorFWCen_RobCen.fpLength;
	//            pstFW->stPot.fpPosX = temp_x_dt35 + (-sinf(-FW_rob_Alpha) + sinf(-FW_rob_Alpha - fpQ_now)) * pstFW->stVectorFWCen_RobCen.fpLength;
	//            pstFW->stPot.fpPosX1 = temp_x_dt35 + (-sinf(-FW_rob_Alpha) + sinf(-FW_rob_Alpha - fpQ_now)) * pstFW->stVectorFWCen_RobCen.fpLength;
	//        }
				if(flag_y_dt35)//非耦合时可用
				{
					temp_x_dt35 = -p_dt35_now->robot_y;
					pstFW->stPot.fpPosX = temp_x_dt35 + (-sinf(-FW_rob_Alpha) + sinf(-FW_rob_Alpha - fpQ_now)) * pstFW->stVectorFWCen_RobCen.fpLength;
          pstFW->stPot.fpPosX1 = temp_x_dt35 + (-sinf(-FW_rob_Alpha) + sinf(-FW_rob_Alpha - fpQ_now)) * pstFW->stVectorFWCen_RobCen.fpLength;
				}
				if(flag_x_dt35)
				{
					temp_y_dt35 = p_dt35_now->robot_x;
					pstFW->stPot.fpPosY = temp_y_dt35 - (cosf(-FW_rob_Alpha) - cosf(-FW_rob_Alpha - fpQ_now)) * pstFW->stVectorFWCen_RobCen.fpLength;
          pstFW->stPot.fpPosY1 = temp_y_dt35 - (cosf(-FW_rob_Alpha) - cosf(-FW_rob_Alpha - fpQ_now)) * pstFW->stVectorFWCen_RobCen.fpLength;
				}
		}
    else /*if (nav.state == NAV_AUTO_PATH)*/
    {
        flag_x_dt35 = 1;
        flag_y_dt35 = 1;
        if (fabs(p_dt35_now->robot_x - pstRobot->stPot.fpPosX) <= 30.0 && fabs(p_dt35_now->pro_x1 - p_dt35_now->pro_x2) <= 40.0) //防止阶跃
        {
            p_dt35_now->robot_x = p_dt35_now->robot_x;
        }
        else if (fabs(p_dt35_now->robot_x1 - pstRobot->stPot.fpPosX) <= 50.0)
        {
            p_dt35_now->robot_x = p_dt35_now->robot_x1;
        }
        else if (fabs(p_dt35_now->robot_x2 - pstRobot->stPot.fpPosX) <= 50.0)
        {
            p_dt35_now->robot_x = p_dt35_now->robot_x2;
        }
        else
        {
            flag_x_dt35 = 0;
        }

        if (fabs(pstRobot->stPot.fpPosX) < 1000 && fabs(pstRobot->stPot.fpPosQ) / 10.0f < 50.0f)
        {
            if (fabs(p_dt35_now->robot_y - pstRobot->stPot.fpPosY) <= 40.0 && fabs(p_dt35_now->pro_y1 - p_dt35_now->pro_y2) <= 60.0)
            {
                p_dt35_now->robot_y = p_dt35_now->robot_y;
            }
            else if (fabs(p_dt35_now->robot_y1 - pstRobot->stPot.fpPosY) <= 40.0)
            {
                p_dt35_now->robot_y = p_dt35_now->robot_y1;
            }
            else if (fabs(p_dt35_now->robot_y2 - pstRobot->stPot.fpPosY) <= 40.0)
            {
                p_dt35_now->robot_y = p_dt35_now->robot_y2;
            }
            else
            {
                flag_y_dt35 = 0;
            }
        }
        else if (fabs(pstRobot->stPot.fpPosX) > 1000&&fabs(pstRobot->stPot.fpPosQ) / 10.0f < 50.0f)
        {
            if (fabs(p_dt35_now->robot_y - pstRobot->stPot.fpPosY + HIDE_Y) <= 40.0 && fabs(p_dt35_now->pro_y1 - p_dt35_now->pro_y2) <= 60.0)
            {
                p_dt35_now->robot_y = p_dt35_now->robot_y + HIDE_Y;
            }
            else if (fabs(p_dt35_now->robot_y1 - pstRobot->stPot.fpPosY + HIDE_Y) <= 40.0)
            {
                p_dt35_now->robot_y = p_dt35_now->robot_y1 + HIDE_Y;
            }
            else if (fabs(p_dt35_now->robot_y2 - pstRobot->stPot.fpPosY + HIDE_Y) <= 40.0)
            {
                p_dt35_now->robot_y = p_dt35_now->robot_y2 + HIDE_Y;
            }
            else
            {
                flag_y_dt35 = 0;
            }
        }
				else
				{
					flag_y_dt35 = 0;
				}

	//        if (flag_x_dt35 && flag_y_dt35)
	//        {
	//					  temp_x_dt35 = -p_dt35_now->robot_y;
	//            temp_y_dt35 = p_dt35_now->robot_x;
	//            pstFW->stPot.fpPosY = temp_y_dt35 - (cosf(-FW_rob_Alpha) - cosf(-FW_rob_Alpha - fpQ_now)) * pstFW->stVectorFWCen_RobCen.fpLength;
	//            pstFW->stPot.fpPosY1 = temp_y_dt35 - (cosf(-FW_rob_Alpha) - cosf(-FW_rob_Alpha - fpQ_now)) * pstFW->stVectorFWCen_RobCen.fpLength;
	//            pstFW->stPot.fpPosX = temp_x_dt35 + (-sinf(-FW_rob_Alpha) + sinf(-FW_rob_Alpha - fpQ_now)) * pstFW->stVectorFWCen_RobCen.fpLength;
	//            pstFW->stPot.fpPosX1 = temp_x_dt35 + (-sinf(-FW_rob_Alpha) + sinf(-FW_rob_Alpha - fpQ_now)) * pstFW->stVectorFWCen_RobCen.fpLength;
	//        }
				if(flag_y_dt35)//非耦合时可用
				{
					temp_x_dt35 = -p_dt35_now->robot_y;
					pstFW->stPot.fpPosX = temp_x_dt35 + (-sinf(-FW_rob_Alpha) + sinf(-FW_rob_Alpha - fpQ_now)) * pstFW->stVectorFWCen_RobCen.fpLength;
          pstFW->stPot.fpPosX1 = temp_x_dt35 + (-sinf(-FW_rob_Alpha) + sinf(-FW_rob_Alpha - fpQ_now)) * pstFW->stVectorFWCen_RobCen.fpLength;
				}
				if(flag_x_dt35)
				{
					temp_y_dt35 = p_dt35_now->robot_x;
					pstFW->stPot.fpPosY = temp_y_dt35 - (cosf(-FW_rob_Alpha) - cosf(-FW_rob_Alpha - fpQ_now)) * pstFW->stVectorFWCen_RobCen.fpLength;
          pstFW->stPot.fpPosY1 = temp_y_dt35 - (cosf(-FW_rob_Alpha) - cosf(-FW_rob_Alpha - fpQ_now)) * pstFW->stVectorFWCen_RobCen.fpLength;
				}
    }
}

extern uint16_t dt35_up, dt35_down, dt35_right, dt35_left;
extern uint16_t dt35_upleft, dt35_upright;
extern fp32 wood_pos_x;

int reloc_times;

float DT35_Q;
void DT35_Q_relocation(dt35_t *p_dt35_now)
{
    p_dt35_now->dt35_fix_x1 = fabs(K_DT35_X1 * p_dt35_now->dt35_x1 + B_DT35_X1);
    p_dt35_now->dt35_fix_x2 = fabs(K_DT35_X2 * p_dt35_now->dt35_x2 + B_DT35_X2);

    DT35_Q = atan2f(p_dt35_now->dt35_fix_x1 - p_dt35_now->dt35_fix_x2, DIS_X1_DT35_TO_C1 + DIS_X2_DT35_TO_C1) / RADIAN;
}

//void Vision_relocation_RED(ST_ROBOT *pstRobot, ST_FOLLOWER_WHEEL *pstFW,tower *p_tower)
//{
//	static float robot_x,robot_y,temp_x,temp_y,fpQ_now;
//	static bool flag_vision=0;
//	float Q_test=(pstRobot->stPot.fpPosQ /10.0f);
//	Q_test= ((int32_t)Q_test+720000)%360+Q_test-(int32_t)Q_test;
//	
//	flag_vision=1;
//	
//	if(p_tower->if_find)
//	{
//		//有效范围应该要调
//		if(fabs(Q_test-RT_LFET_Q)<20||fabs(Q_test-RT_LFET_Q-360)<20)
//		{
//			pstRobot->stPot.fpPosQ1=10.0f*(RT_LFET_Q + p_tower->q - fpQOffset);
//			pstRobot->stPot.fpPosQ=10.0f*(RT_LFET_Q + p_tower->q);
//			robot_x = RT_LEFT_X - (p_tower->y+CAMERA_X)*cosf(p_tower->q*RADIAN)-(p_tower->x+CAMERA_Y)*sinf(p_tower->q*RADIAN);
//			robot_y = RT_LEFT_Y - (p_tower->y+CAMERA_X)*sinf(p_tower->q*RADIAN)+(p_tower->x+CAMERA_Y)*cosf(p_tower->q*RADIAN);
//		}
//		else if(fabs(Q_test-RT_UP_Q)<20||fabs(Q_test-RT_UP_Q-360)<20)
//		{
//			pstRobot->stPot.fpPosQ1=10.0f*(RT_UP_Q + p_tower->q - fpQOffset);
//			pstRobot->stPot.fpPosQ=10.0f*(RT_UP_Q + p_tower->q);
//			robot_x = RT_UP_X - (p_tower->y+CAMERA_X)*sinf(p_tower->q*RADIAN)+(p_tower->x+CAMERA_Y)*cosf(p_tower->q*RADIAN);
//			robot_y = RT_UP_Y + (p_tower->y+CAMERA_X)*cosf(p_tower->q*RADIAN)+(p_tower->x+CAMERA_Y)*sinf(p_tower->q*RADIAN);
//		}
//		else if(fabs(Q_test-RT_DOWN_Q)<20||fabs(Q_test-RT_DOWN_Q-360)<20)
//		{
//			pstRobot->stPot.fpPosQ1=10.0f*(RT_DOWN_Q + p_tower->q - fpQOffset);
//			pstRobot->stPot.fpPosQ=10.0f*(RT_DOWN_Q + p_tower->q);
//			robot_x = RT_DOWN_X + (p_tower->y+CAMERA_X)*sinf(p_tower->q*RADIAN)-(p_tower->x+CAMERA_Y)*cosf(p_tower->q*RADIAN);
//			robot_y = RT_DOWN_Y - (p_tower->y+CAMERA_X)*cosf(p_tower->q*RADIAN)-(p_tower->x+CAMERA_Y)*sinf(p_tower->q*RADIAN);
//		}
//		else
//		{
//			flag_vision=0;
//		}
//		
//		if(flag_vision)
//		{
//			reloc_times++;
//			fpQ_now=pstRobot->stPot.fpPosQ*RADIAN;
//			
//			temp_x = -robot_y;
//			temp_y = robot_x;
//			
//			pstFW->stPot.fpPosY = temp_y - (cosf(-FW_rob_Alpha) - cosf(-FW_rob_Alpha - fpQ_now)) * pstFW->stVectorFWCen_RobCen.fpLength;
//			pstFW->stPot.fpPosY1 = temp_y - (cosf(-FW_rob_Alpha) - cosf(-FW_rob_Alpha - fpQ_now)) * pstFW->stVectorFWCen_RobCen.fpLength;
//			pstFW->stPot.fpPosX = temp_x + (-sinf(-FW_rob_Alpha) + sinf(-FW_rob_Alpha - fpQ_now)) * pstFW->stVectorFWCen_RobCen.fpLength;
//			pstFW->stPot.fpPosX1 = temp_x + (-sinf(-FW_rob_Alpha) + sinf(-FW_rob_Alpha - fpQ_now)) * pstFW->stVectorFWCen_RobCen.fpLength;
//		}
//	}
//}

//void Vision_relocation_BLUE(ST_ROBOT *pstRobot, ST_FOLLOWER_WHEEL *pstFW,tower *p_tower)
//{
//	static float robot_x,robot_y,temp_x,temp_y,fpQ_now;
//	static bool flag_vision=0;
//	float Q_test=(pstRobot->stPot.fpPosQ /10.0f);
//	Q_test= ((int32_t)Q_test+720000)%360+Q_test-(int32_t)Q_test;
//	flag_vision=1;
//	
//	if(p_tower->if_find)
//	{
//		//有效范围应该要调
//		if(fabs(Q_test-LT_RIGHT_Q)<20||fabs(Q_test-LT_RIGHT_Q-360)<20)
//		{
//			pstRobot->stPot.fpPosQ1=10.0f*(LT_RIGHT_Q + p_tower->q - fpQOffset);
//			pstRobot->stPot.fpPosQ=10.0f*(LT_RIGHT_Q + p_tower->q);
//			robot_x = LT_RIGHT_X + (p_tower->y+CAMERA_X)*cosf(p_tower->q*RADIAN)+(p_tower->x+CAMERA_Y)*sinf(p_tower->q*RADIAN);
//			robot_y = LT_RIGHT_Y + (p_tower->y+CAMERA_X)*sinf(p_tower->q*RADIAN)-(p_tower->x+CAMERA_Y)*cosf(p_tower->q*RADIAN);
//		}
//		else if(fabs(Q_test-LT_UP_Q)<20||fabs(Q_test-LT_UP_Q-360)<20)
//		{
//			pstRobot->stPot.fpPosQ1=10.0f*(LT_UP_Q + p_tower->q - fpQOffset);
//			pstRobot->stPot.fpPosQ=10.0f*(LT_UP_Q + p_tower->q);
//			robot_x = LT_UP_X - (p_tower->y+CAMERA_X)*sinf(p_tower->q*RADIAN)+(p_tower->x+CAMERA_Y)*cosf(p_tower->q*RADIAN);
//			robot_y = LT_UP_Y + (p_tower->y+CAMERA_X)*cosf(p_tower->q*RADIAN)+(p_tower->x+CAMERA_Y)*sinf(p_tower->q*RADIAN);
//		}
//		else if(fabs(Q_test-LT_DOWN_Q)<20||fabs(Q_test-LT_DOWN_Q-360)<20)
//		{
//			pstRobot->stPot.fpPosQ1=10.0f*(LT_DOWN_Q + p_tower->q - fpQOffset);
//			pstRobot->stPot.fpPosQ=10.0f*(LT_DOWN_Q + p_tower->q);
//			robot_x = LT_DOWN_X + (p_tower->y+CAMERA_X)*sinf(p_tower->q*RADIAN)-(p_tower->x+CAMERA_Y)*cosf(p_tower->q*RADIAN);
//			robot_y = LT_DOWN_Y - (p_tower->y+CAMERA_X)*cosf(p_tower->q*RADIAN)-(p_tower->x+CAMERA_Y)*sinf(p_tower->q*RADIAN);
//		}
//		else
//		{
//			flag_vision=0;
//		}
//		
//		if(flag_vision)
//		{
//			fpQ_now=pstRobot->stPot.fpPosQ*RADIAN;
//			
//			temp_x = -robot_y;
//			temp_y = robot_x;
//			
//			pstFW->stPot.fpPosY = temp_y - (cosf(-FW_rob_Alpha) - cosf(-FW_rob_Alpha - fpQ_now)) * pstFW->stVectorFWCen_RobCen.fpLength;
//			pstFW->stPot.fpPosY1 = temp_y - (cosf(-FW_rob_Alpha) - cosf(-FW_rob_Alpha - fpQ_now)) * pstFW->stVectorFWCen_RobCen.fpLength;
//			pstFW->stPot.fpPosX = temp_x + (-sinf(-FW_rob_Alpha) + sinf(-FW_rob_Alpha - fpQ_now)) * pstFW->stVectorFWCen_RobCen.fpLength;
//			pstFW->stPot.fpPosX1 = temp_x + (-sinf(-FW_rob_Alpha) + sinf(-FW_rob_Alpha - fpQ_now)) * pstFW->stVectorFWCen_RobCen.fpLength;
//		}
//	}
//}

int DT35_X1,DT35_X2,DT35_X;
int dt35_movemode;

void DT35_Boundary_Blue_1(ST_ROBOT *pstRobot, ST_FOLLOWER_WHEEL *pstFW, dt35_t *p_dt35_now)
{
  p_dt35_now->dt35_fix_y1 = fabs(K_DT35_Y1 * p_dt35_now->dt35_y1 + B_DT35_Y1);
  p_dt35_now->dt35_fix_y2 = fabs(K_DT35_Y2 * p_dt35_now->dt35_y2 + B_DT35_Y2);
	  
	DT35_Q = atan2f(-p_dt35_now->dt35_fix_y1 + p_dt35_now->dt35_fix_y2, DIS_Y1_DT35_TO_C1 + DIS_Y2_DT35_TO_C1) / RADIAN;
	pstRobot->stPot.fpPosQ1=10.0f*(270.0f + DT35_Q);
	pstRobot->stPot.fpPosQ=10.0f*(270.0f + DT35_Q);
	
  DT35_X1 = p_dt35_now->dt35_fix_y1 * cosf(DT35_Q*RADIAN) + DIS_Y1_DT35_TO_C1 * sinf(DT35_Q*RADIAN) + DIS_Y1_DT35_TO_C2 * cosf(DT35_Q*RADIAN);
  DT35_X2 = p_dt35_now->dt35_fix_y2 * cosf(DT35_Q*RADIAN) - DIS_Y2_DT35_TO_C1 * sinf(DT35_Q*RADIAN) + DIS_Y2_DT35_TO_C2 * cosf(DT35_Q*RADIAN);
	
	DT35_X=(DT35_X1+DT35_X2)/2.0f;
	
//	point_end.point_set(pstRobot->stPot.fpPosX - DT35_X + DIS_X_LEFT,pstRobot->stPot.fpPosY,270.0f);
	dt35_movemode=1;
	SET_NAV_PATH_AUTO(1);
}
