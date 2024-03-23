#include "imu_update.h"
#include "math.h"

_imu_st imu_data =  {1,0,0,0,0,0,
    {0,0,0},
    {0,0,0},
    {0,0,0},
    {0,0,0},
    {0,0,0},
    {0,0,0},
    0,0,0
};

extern _sensor_st sensor;


extern FP32 Gyro_X_Real;
extern FP32 Gyro_Y_Real;
extern FP32 Gyro_Z_Real;

extern FP32 Acc_X_Real;
extern FP32 Acc_Y_Real;
extern FP32 Acc_Z_Real;

float Kp = 0.5f;/**/
float Ki = 0.001f;/**/
//float Kp = 0.0f;/**/
//float Ki = 0.000f;/**/
/**/
float exInt = 0.0f;
float eyInt = 0.0f;
float ezInt = 0.0f;
/**/
static float q0 = 1.0f;
static float q1 = 0.0f;
static float q2 = 0.0f;
static float q3 = 0.0f;

float q0_send,q1_send,q2_send,q3_send;//给视觉相机标定用发的四元数数据

void IMU_Update_Mahony(_imu_st *imu,float dt)
  	// 包含了IMU的数据，包括加速度计和陀螺仪数据，以及计算得到的欧拉角。在这段代码中，它可以存储计算的结果。
{   // 定义一些需要用到的变量，包括正常化的因子，加速度计数据的正常化向量，误差向量的单位，上一个四元数
    // 的值，以及一些常量和临时变量。
	
    float normalise; //代表四元数正规化的因子，用于确保四元数是单位四元数，其值为四元数各元素平方和的逆开方。
    float nor_acc[VEC_XYZ] = {0};
    float ex, ey, ez;//
    float q0s, q1s, q2s, q3s;/* 代表当前时刻的四元数。四元数是表示三维旋转的一种方式，可以避免欧拉角的锁死现象。 */
    static float R11,R21;/* (1,1),(2,1) */   //这些变量代表旋转矩阵的元素，用于计算偏航角。
    static float vecxZ, vecyZ, veczZ;/* z(0,0,1)' */    //代表重力向量在机体坐标系的分量，用于计算俯仰角和横滚角。
    float half_T = 0.5f * dt;

    float q0Last = q0;
    float q1Last = q1;
    float q2Last = q2;
    float q3Last = q3;     //这四个变量代表上一时刻的四元数的值，用于计算当前时刻四元数的值。
    float delta_theta[3];/* xyz */   //代表陀螺仪测得的角速度Δt，用于计算四元数的更新。
    float delta_theta_s;/* xyz */    //代表陀螺仪输出值的变化率。

    /* 0 */
		// 检查加速度计的值，如果加速度计的值不为零，说明物体处于运动状态,
		//这里的三个real已经经过了低通滤波处理
    if((Acc_X_Real != 0.0f) || (Acc_Y_Real != 0.0f) || (Acc_Z_Real != 0.0f))
    {
			  //获取每个加速度计的读数，并存入正常化向量中
        nor_acc[X] = Acc_X_Real;
        nor_acc[Y] = Acc_Y_Real;
        nor_acc[Z] = Acc_Z_Real;

        /*用取逆平方根函数对加速度向量进行正常化，得到单位向量。 */
        normalise = invSqrt(nor_acc[X] * nor_acc[X] + nor_acc[Y] * nor_acc[Y] + nor_acc[Z] * nor_acc[Z]);
        nor_acc[X] *= normalise;
        nor_acc[Y] *= normalise;
        nor_acc[Z] *= normalise;

        /* 计算预期向量和实际向量的误差。 */
        /* |a x b| = |a|*|b|*sin(theta);|a|=|b|=1,thetasin(theta)theta, */
        ex = (nor_acc[Y] * veczZ - nor_acc[Z] * vecyZ);
        ey = (nor_acc[Z] * vecxZ - nor_acc[X] * veczZ);
        ez = (nor_acc[X] * vecyZ - nor_acc[Y] * vecxZ);

        /* 计算每个误差向量的累积，这将用于后面的积分部分 */
        exInt += Ki * ex * dt ;
        eyInt += Ki * ey * dt ;
        ezInt += Ki * ez * dt ;

        /* PI, */ // //将积分误差添加到陀螺仪值以改进后面的输出。
        Gyro_X_Real += Kp * ex + exInt;
        Gyro_Y_Real += Kp * ey + eyInt;
        Gyro_Z_Real += Kp * ez + ezInt;
    }
		
		//比例因子half_T对应到离散时间积分的一部分，它实际上把角速度转换到了角度，换言之就是关于dt的一阶泰勒展开
    /* TkTk+1, */
    delta_theta[0] = Gyro_X_Real*half_T;
    delta_theta[1] = Gyro_Y_Real*half_T;
    delta_theta[2] = Gyro_Z_Real*half_T;
    delta_theta_s = delta_theta[0]*delta_theta[0] + delta_theta[1]*delta_theta[1] + delta_theta[2]*delta_theta[2];
    /* 更新了四元数的值。四元数用于表示旋转，这里的更新是基于陀螺仪的角速度测量。 */
    /* Q(Tk+1)=(I+0.5*delta_theta)Q(Tk) */
// 	q0 += -q1Last * delta_theta[0] - q2Last * delta_theta[1] - q3Last * delta_theta[2];
// 	q1 +=  q0Last * delta_theta[0] + q2Last * delta_theta[2] - q3Last * delta_theta[1];
// 	q2 +=  q0Last * delta_theta[1] - q1Last * delta_theta[2] + q3Last * delta_theta[0];
// 	q3 +=  q0Last * delta_theta[2] + q1Last * delta_theta[1] - q2Last * delta_theta[0];

    /* 这里是另一种四元数更新方式，考虑了角速度变化的大小(delta_theta_s)来对四元数进行调整，更好一些*/
    /* Q(Tk+1)=((1-0.125*delta_theta_s)I+0.5*delta_theta)Q(Tk) */
    q0 = q0Last*(1-delta_theta_s) - q1Last * delta_theta[0] - q2Last * delta_theta[1] - q3Last * delta_theta[2];
    q1 = q1Last*(1-delta_theta_s) + q0Last * delta_theta[0] + q2Last * delta_theta[2] - q3Last * delta_theta[1];
    q2 = q2Last*(1-delta_theta_s) + q0Last * delta_theta[1] - q1Last * delta_theta[2] + q3Last * delta_theta[0];
    q3 = q3Last*(1-delta_theta_s) + q0Last * delta_theta[2] + q1Last * delta_theta[1] - q2Last * delta_theta[0];
    /*  正规化四元数 
   此步骤保证四元数保持单位长度，这对于表示旋转是必要的。 */
    normalise = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= normalise;
    q1 *= normalise;
    q2 *= normalise;
    q3 *= normalise;
		/*2024.01.24加，为了给视觉反馈四元数信息*/
		q0_send = q0;
		q1_send = q1;
		q2_send = q2;
		q3_send = q3;
    /* 计算四元数各分量的平方 */ 
    q0s = q0 * q0;
    q1s = q1 * q1;
    q2s = q2 * q2;
    q3s = q3 * q3;
		/* 计算旋转矩阵的部分元素 */
    // 这些元素将用于计算欧拉角。
    R11 = q0s + q1s - q2s - q3s;/* (1,1) */
    R21 = 2 * (q1 * q2 + q0 * q3);/* (2,1) */
		/* 计算重力向量在机体坐标系的分量 */
    // 这将用于计算俯仰角和横滚角。
    /* z(0,0,1) */
    vecxZ = 2 * (q1 * q3 - q0 * q2);/* (3,1) */
    vecyZ = 2 * (q0 * q1 + q2 * q3);/* (3,2) */
    veczZ = q0s - q1s - q2s + q3s;	/* (3,3) */
		/* 限制vecxZ的值在[-1,1]之间，以确保asinf函数的输入有效 */
    if (vecxZ>1) vecxZ=1;
    if (vecxZ<-1) vecxZ=-1;
		/* 计算并更新IMU的欧拉角 */
    // 这些角度通常用于方便地表示和理解物体的方向。
    /* roll pitch yaw  */
    imu->pit = -asinf(vecxZ) * 57.30f;       //俯仰角pitch--弧度转换为角度
    imu->rol = atan2f(vecyZ, veczZ) * 57.30f;    //翻滚角roll
    imu->yaw = atan2f(R21, R11) * 57.30f;      //偏航角yaw

}


