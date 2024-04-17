#include "gimbal_app.h"


/*----------------------------------------------------------------------------------------
函数名：GimbalControl()
功能： 云台闭环控制
----------------------------------------------------------------------------------------*/

extern _imu_st imu_data;
extern FP32 Encoder_pitch;

SSHORT16 G_Compensate = 8000;
SSHORT16 Gimmbal_Roll_Cnt = 0;
bool Shoot_Flag = FALSE;

bool Friction_Control;

void Gimbal_Control(void)
{
    static FP32 Pre_imu_data_Yaw;
    static UCHAR8 Clk_Div = 0;

    Clk_Div++;
    /*Set Speed Reference*/
    g_stGM_PitchSpeedPID.fpDes = g_stGM_PitchPosPID.fpU;
    g_stGM_YawSpeedPID.fpDes = g_stGM_YawPosPID.fpU;

    /*Get Speed Feedback*/
    g_stGM_PitchSpeedPID.fpFB = -sensor.Gyro_deg[0];
    g_stGM_YawSpeedPID.fpFB = sensor.Gyro_deg[2];

    /*Speed Loop*/
    CalIWeakenPID(&g_stGM_PitchSpeedPID);
    CalIWeakenPID(&g_stGM_YawSpeedPID);

    if(Clk_Div == 4)
    {
        if(Pre_imu_data_Yaw - imu_data.yaw > 300)
            Gimmbal_Roll_Cnt++;
        else if(Pre_imu_data_Yaw - imu_data.yaw < -300)
            Gimmbal_Roll_Cnt--;

        /*Get Position Feedback*/
//		g_stGM_PitchPosPID.fpFB = Encoder_pitch;
        g_stGM_YawPosPID.fpFB = imu_data.yaw + Gimmbal_Roll_Cnt*360;						//反角度值

        /*Position Loop*/
        CalIWeakenPID(&g_stGM_PitchPosPID);
        CalIWeakenPID(&g_stGM_YawPosPID);

        /*拨弹电机闭环控制*/
        CalIResistedPID(&g_stShooterPosPID);
        g_stShooterSpeedPID.fpDes = g_stShooterPosPID.fpU/8192 * 360.0f;
        CalIWeakenPID(&g_stShooterSpeedPID);

        TIM_SetCompare4(TIM1,20000-2*Friction_Spd1);
        TIM_SetCompare3(TIM1,20000-2*Friction_Spd2);

        if(Friction_Spd1 == 900)
            g_stShooterSpeedPID.fpUMax = 10000;
        else
            g_stShooterSpeedPID.fpUMax = 0;

        /*安全模式*/
        if(Friction_Spd1 < 500)
        {
            g_stGM_PitchSpeedPID.fpUMax = 0;
            g_stGM_YawSpeedPID.fpUMax = 0;
            g_stShooterSpeedPID.fpUMax  = 0;
        }
        else
        {
            g_stGM_PitchSpeedPID.fpUMax = 30000;
            g_stGM_YawSpeedPID.fpUMax = 32767;
            g_stShooterSpeedPID.fpUMax  = 10000;

        }

        Send_Current_To_Gimbal(CAN1,	(SSHORT16)(-g_stGM_YawSpeedPID.fpU), (SSHORT16)(g_stGM_PitchSpeedPID.fpU + G_Compensate * cos(g_stGM_PitchPosPID.fpFB/360*2*PI)));
        //Send_Current_To_Gimbal(CAN1,0,0,0);//(SSHORT16)(g_stShooterSpeedPID.fpU));

        Pre_imu_data_Yaw = imu_data.yaw;
        Clk_Div = 0;
    }
}
