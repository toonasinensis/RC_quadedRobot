#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__

#include "main.h"
#include "gimbal_control_types.h"

#define PitchPosFB_Gyro         -imu_data.rol                                   //陀螺仪位置反馈，这里的rol就是值pit，为了防止坐标变换错误，就直接用不改名了
#define YawPosFB_Gyro           Angle_180_To_Inf(imu_data.yaw, &G_ST_IMU_Angle) //陀螺仪位置反馈
#define PitchPosFB_Encoder      -Angle_Inf_To_180(Pitch_Angle-PitchEncoderZero) //编码器位置反馈
#define YawPosFB_Encoder_Norm   Angle_180_To_Inf(Yaw_Angle-YawEncoderZero_Norm, &G_ST_YawEncoder_Norm_Angle) //编码器位置反馈，正常模式
#define YawPosFB_Encoder_Buff   Angle_180_To_Inf(Yaw_Angle-YawEncoderZero_Buff, &G_ST_YawEncoder_Buff_Angle) //编码器位置反馈，大幅模式
#define PitchSpeedFB            -Gyro_X_Ori                              //陀螺仪速度反馈
#define YawSpeedFB              Gyro_Z_Ori                              //陀螺仪速度反馈

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
