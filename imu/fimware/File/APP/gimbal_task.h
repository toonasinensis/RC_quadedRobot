#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__

#include "main.h"
#include "gimbal_control_types.h"

#define PitchPosFB_Gyro         -imu_data.rol                                   //������λ�÷����������rol����ֵpit��Ϊ�˷�ֹ����任���󣬾�ֱ���ò�������
#define YawPosFB_Gyro           Angle_180_To_Inf(imu_data.yaw, &G_ST_IMU_Angle) //������λ�÷���
#define PitchPosFB_Encoder      -Angle_Inf_To_180(Pitch_Angle-PitchEncoderZero) //������λ�÷���
#define YawPosFB_Encoder_Norm   Angle_180_To_Inf(Yaw_Angle-YawEncoderZero_Norm, &G_ST_YawEncoder_Norm_Angle) //������λ�÷���������ģʽ
#define YawPosFB_Encoder_Buff   Angle_180_To_Inf(Yaw_Angle-YawEncoderZero_Buff, &G_ST_YawEncoder_Buff_Angle) //������λ�÷��������ģʽ
#define PitchSpeedFB            -Gyro_X_Ori                              //�������ٶȷ���
#define YawSpeedFB              Gyro_Z_Ori                              //�������ٶȷ���

typedef struct
{
    float angle_180;
    float angle_180_pre;
    float angle_inf;
} ST_ANGLE;

typedef enum
{
    PITCH_FORWARD,
    PITCH_STATIC,
    PITCH_BACKWARD,
} PITCHSTATUS;

void GimbalControl(void);
extern ST_ANGLE G_ST_IMU_Angle;
extern float Angle_180_To_Inf(float angle_input, ST_ANGLE* st_angle);
#endif
