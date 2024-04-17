#include "gimbal_task.h"


ST_ANGLE G_ST_IMU_Angle = {0};
ST_ANGLE G_ST_YawEncoder_Norm_Angle = {0};
ST_ANGLE G_ST_YawEncoder_Buff_Angle = {0};
float Angle_180_To_Inf(float angle_input, ST_ANGLE* st_angle)
{
    st_angle->angle_180 = angle_input;

    if( st_angle->angle_180_pre - st_angle->angle_180 > 180 )
        st_angle->angle_inf += (st_angle->angle_180 - st_angle->angle_180_pre) + 360;
    else if( st_angle->angle_180 - st_angle->angle_180_pre > 180 )
        st_angle->angle_inf += (st_angle->angle_180 - st_angle->angle_180_pre) - 360;
    else
        st_angle->angle_inf += (st_angle->angle_180 - st_angle->angle_180_pre);

    st_angle->angle_180_pre = angle_input;

    return st_angle->angle_inf;
}

