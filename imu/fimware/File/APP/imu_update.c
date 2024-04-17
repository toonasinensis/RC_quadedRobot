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

float q0_send,q1_send,q2_send,q3_send;//���Ӿ�����궨�÷�����Ԫ������

void IMU_Update_Mahony(_imu_st *imu,float dt)
  	// ������IMU�����ݣ��������ٶȼƺ����������ݣ��Լ�����õ���ŷ���ǡ�����δ����У������Դ洢����Ľ����
{   // ����һЩ��Ҫ�õ��ı��������������������ӣ����ٶȼ����ݵ���������������������ĵ�λ����һ����Ԫ��
    // ��ֵ���Լ�һЩ��������ʱ������
	
    float normalise; //������Ԫ�����滯�����ӣ�����ȷ����Ԫ���ǵ�λ��Ԫ������ֵΪ��Ԫ����Ԫ��ƽ���͵��濪����
    float nor_acc[VEC_XYZ] = {0};
    float ex, ey, ez;//
    float q0s, q1s, q2s, q3s;/* ����ǰʱ�̵���Ԫ������Ԫ���Ǳ�ʾ��ά��ת��һ�ַ�ʽ�����Ա���ŷ���ǵ��������� */
    static float R11,R21;/* (1,1),(2,1) */   //��Щ����������ת�����Ԫ�أ����ڼ���ƫ���ǡ�
    static float vecxZ, vecyZ, veczZ;/* z(0,0,1)' */    //�������������ڻ�������ϵ�ķ��������ڼ��㸩���Ǻͺ���ǡ�
    float half_T = 0.5f * dt;

    float q0Last = q0;
    float q1Last = q1;
    float q2Last = q2;
    float q3Last = q3;     //���ĸ�����������һʱ�̵���Ԫ����ֵ�����ڼ��㵱ǰʱ����Ԫ����ֵ��
    float delta_theta[3];/* xyz */   //���������ǲ�õĽ��ٶȦ�t�����ڼ�����Ԫ���ĸ��¡�
    float delta_theta_s;/* xyz */    //�������������ֵ�ı仯�ʡ�

    /* 0 */
		// �����ٶȼƵ�ֵ��������ٶȼƵ�ֵ��Ϊ�㣬˵�����崦���˶�״̬,
		//���������real�Ѿ������˵�ͨ�˲�����
    if((Acc_X_Real != 0.0f) || (Acc_Y_Real != 0.0f) || (Acc_Z_Real != 0.0f))
    {
			  //��ȡÿ�����ٶȼƵĶ�����������������������
        nor_acc[X] = Acc_X_Real;
        nor_acc[Y] = Acc_Y_Real;
        nor_acc[Z] = Acc_Z_Real;

        /*��ȡ��ƽ���������Լ��ٶ������������������õ���λ������ */
        normalise = invSqrt(nor_acc[X] * nor_acc[X] + nor_acc[Y] * nor_acc[Y] + nor_acc[Z] * nor_acc[Z]);
        nor_acc[X] *= normalise;
        nor_acc[Y] *= normalise;
        nor_acc[Z] *= normalise;

        /* ����Ԥ��������ʵ���������� */
        /* |a x b| = |a|*|b|*sin(theta);|a|=|b|=1,thetasin(theta)theta, */
        ex = (nor_acc[Y] * veczZ - nor_acc[Z] * vecyZ);
        ey = (nor_acc[Z] * vecxZ - nor_acc[X] * veczZ);
        ez = (nor_acc[X] * vecyZ - nor_acc[Y] * vecxZ);

        /* ����ÿ������������ۻ����⽫���ں���Ļ��ֲ��� */
        exInt += Ki * ex * dt ;
        eyInt += Ki * ey * dt ;
        ezInt += Ki * ez * dt ;

        /* PI, */ // //�����������ӵ�������ֵ�ԸĽ�����������
        Gyro_X_Real += Kp * ex + exInt;
        Gyro_Y_Real += Kp * ey + eyInt;
        Gyro_Z_Real += Kp * ez + ezInt;
    }
		
		//��������half_T��Ӧ����ɢʱ����ֵ�һ���֣���ʵ���ϰѽ��ٶ�ת�����˽Ƕȣ�����֮���ǹ���dt��һ��̩��չ��
    /* TkTk+1, */
    delta_theta[0] = Gyro_X_Real*half_T;
    delta_theta[1] = Gyro_Y_Real*half_T;
    delta_theta[2] = Gyro_Z_Real*half_T;
    delta_theta_s = delta_theta[0]*delta_theta[0] + delta_theta[1]*delta_theta[1] + delta_theta[2]*delta_theta[2];
    /* ��������Ԫ����ֵ����Ԫ�����ڱ�ʾ��ת������ĸ����ǻ��������ǵĽ��ٶȲ����� */
    /* Q(Tk+1)=(I+0.5*delta_theta)Q(Tk) */
// 	q0 += -q1Last * delta_theta[0] - q2Last * delta_theta[1] - q3Last * delta_theta[2];
// 	q1 +=  q0Last * delta_theta[0] + q2Last * delta_theta[2] - q3Last * delta_theta[1];
// 	q2 +=  q0Last * delta_theta[1] - q1Last * delta_theta[2] + q3Last * delta_theta[0];
// 	q3 +=  q0Last * delta_theta[2] + q1Last * delta_theta[1] - q2Last * delta_theta[0];

    /* ��������һ����Ԫ�����·�ʽ�������˽��ٶȱ仯�Ĵ�С(delta_theta_s)������Ԫ�����е���������һЩ*/
    /* Q(Tk+1)=((1-0.125*delta_theta_s)I+0.5*delta_theta)Q(Tk) */
    q0 = q0Last*(1-delta_theta_s) - q1Last * delta_theta[0] - q2Last * delta_theta[1] - q3Last * delta_theta[2];
    q1 = q1Last*(1-delta_theta_s) + q0Last * delta_theta[0] + q2Last * delta_theta[2] - q3Last * delta_theta[1];
    q2 = q2Last*(1-delta_theta_s) + q0Last * delta_theta[1] - q1Last * delta_theta[2] + q3Last * delta_theta[0];
    q3 = q3Last*(1-delta_theta_s) + q0Last * delta_theta[2] + q1Last * delta_theta[1] - q2Last * delta_theta[0];
    /*  ���滯��Ԫ�� 
   �˲��豣֤��Ԫ�����ֵ�λ���ȣ�����ڱ�ʾ��ת�Ǳ�Ҫ�ġ� */
    normalise = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= normalise;
    q1 *= normalise;
    q2 *= normalise;
    q3 *= normalise;
		/*2024.01.24�ӣ�Ϊ�˸��Ӿ�������Ԫ����Ϣ*/
		q0_send = q0;
		q1_send = q1;
		q2_send = q2;
		q3_send = q3;
    /* ������Ԫ����������ƽ�� */ 
    q0s = q0 * q0;
    q1s = q1 * q1;
    q2s = q2 * q2;
    q3s = q3 * q3;
		/* ������ת����Ĳ���Ԫ�� */
    // ��ЩԪ�ؽ����ڼ���ŷ���ǡ�
    R11 = q0s + q1s - q2s - q3s;/* (1,1) */
    R21 = 2 * (q1 * q2 + q0 * q3);/* (2,1) */
		/* �������������ڻ�������ϵ�ķ��� */
    // �⽫���ڼ��㸩���Ǻͺ���ǡ�
    /* z(0,0,1) */
    vecxZ = 2 * (q1 * q3 - q0 * q2);/* (3,1) */
    vecyZ = 2 * (q0 * q1 + q2 * q3);/* (3,2) */
    veczZ = q0s - q1s - q2s + q3s;	/* (3,3) */
		/* ����vecxZ��ֵ��[-1,1]֮�䣬��ȷ��asinf������������Ч */
    if (vecxZ>1) vecxZ=1;
    if (vecxZ<-1) vecxZ=-1;
		/* ���㲢����IMU��ŷ���� */
    // ��Щ�Ƕ�ͨ�����ڷ���ر�ʾ���������ķ���
    /* roll pitch yaw  */
    imu->pit = -asinf(vecxZ) * 57.30f;       //������pitch--����ת��Ϊ�Ƕ�
    imu->rol = atan2f(vecyZ, veczZ) * 57.30f;    //������roll
    imu->yaw = atan2f(R21, R11) * 57.30f;      //ƫ����yaw

}


