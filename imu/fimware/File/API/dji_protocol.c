#include "dji_protocol.h"

#define TorToCrt (float)32767/0.5f/9.0f

s16 PitchCurrent = 0;
s16 YawCurrent = 0;
s16 ShooterCurrent = 0;
s16 FrictionWheel_Left_Current = 0;
s16 FrictionWheel_Right_Current = 0;

float gravity_torque = 0;//重力矩，通过辨识测出来的

float Pitch_Compensate;
float Yaw_Compensate = 0;
float Shooter_Compensate = 0;
extern s16 Shooter_Torque;



/*************************************************************************
函数功能：接收DJI陀螺仪的数据
*************************************************************************/
float Get_DJI_GYRO(CanRxMsg* rxmessage)
{
    float gyro_temp;
    gyro_temp = 0.01f*((s32)(rxmessage->Data[0]<<24)|(s32)(rxmessage->Data[1]<<16) | (s32)(rxmessage->Data[2]<<8) | (s32)(rxmessage->Data[3]));
    return gyro_temp;
}
