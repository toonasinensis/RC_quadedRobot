#include "para_identify.h"

IDTFY_DATA identify_data[Sample_Num];

mat A;
FP32 A_Data[Sample_Num*Vector_Order];
mat y;
FP32 y_Data[Sample_Num];
mat AT;
FP32 AT_Data[Sample_Num*Vector_Order];
mat x;
FP32 x_Data[Vector_Order];
mat Err;
FP32 Err_Data[Sample_Num];

mat TEMP1,TEMP2,TEMP3,TEMP4;
float       TEMP_data1[Vector_Order*Vector_Order],
            TEMP_data2[Vector_Order*Vector_Order],
            TEMP_data3[Sample_Num*Vector_Order],
            TEMP_data4[Sample_Num];

u8 sample_cnt;
FP32 	    J_Scope_Acc,
            J_Scope_Acc_LPF,
            J_Scope_Spd,
            J_Scope_Pos,
            J_Scope_Cos,
            J_Scope_Sin,
            J_Scope_Input,
            J_Scope_Sign;

extern _imu_st imu_data;
extern CanRxMsg CAN1_RxMsg;

void ParameterInit(void)
{
//    stIdtfyCtrl.fpJpp = 0.00155284093f;
//    stIdtfyCtrl.fpGc = 0.19196482f;
//    stIdtfyCtrl.fpGs = -1.26547122f;
//    stIdtfyCtrl.fpFp = 0.0521783009f;
}

float SineSignal_Output(float A, float f, float offset, float acc_tim, float dt)
{
    static float A_in = 0;
    static u16 tick = 0;
    float out = 0;

    RampSignal(&A_in,A,A*dt/acc_tim);
    out = A_in*sin(2*PI*f*tick*dt) + offset;
    tick++;
    if(tick == 1.0f/dt) tick = 0;
    return out;
}


float State_Sampling(u8 clk_div, float dt)
{
    static float Kt = 1.5*14*0.02385f;
    static float pre_yawspd = 0, pre_pitspd = 0;
    static u8 cnt = 0;

    if(stIdtfyCtrl.emIdtfyMode == Idtfy_yaw)
    {
        J_Scope_Spd = sensor.Gyro_deg[YAW]/57.29578f;
        J_Scope_Acc = (sensor.Gyro_deg[YAW] - pre_yawspd)/57.29578f/dt;
        J_Scope_Sign = Sign_Judge(sensor.Gyro_deg[YAW]);
        J_Scope_Input = -Kt*Yaw_Torque/1000.0f;
        pre_yawspd = sensor.Gyro_deg[YAW];
    }
    else if(stIdtfyCtrl.emIdtfyMode == Idtfy_pitch)
    {
        J_Scope_Spd = g_stGM_PitchSpeedPID.fpFB/57.29578f;
        J_Scope_Acc = (g_stGM_PitchSpeedPID.fpFB - pre_pitspd)/57.29578f/dt;
        J_Scope_Sign = Sign_Judge(g_stGM_PitchSpeedPID.fpFB);
        arm_sin_cos_f32(g_stGM_PitchPosPID.fpFB,&J_Scope_Sin,&J_Scope_Cos);
        J_Scope_Input = -Pitch_Torque;
        pre_pitspd = g_stGM_PitchSpeedPID.fpFB;
    }

    if(stIdtfyCtrl.emSampleState == ENABLE)
    {
        cnt++;
        if(cnt == clk_div)
        {
            identify_data[sample_cnt].Acc = J_Scope_Acc;
            identify_data[sample_cnt].Spd = J_Scope_Spd;
            identify_data[sample_cnt].Cos_theta = J_Scope_Cos;
            identify_data[sample_cnt].Sin_theta = J_Scope_Sin;
            identify_data[sample_cnt].Sign_Spd = J_Scope_Sign;
            identify_data[sample_cnt].Input = J_Scope_Input;
            sample_cnt++;
            cnt = 0;
        }
    }

    if(sample_cnt == Sample_Num)
        return 1;
    else
        return 0;
}

float err_dot = 0;
void CalParameter(void)
{
    u16 i;
    if(stIdtfyCtrl.emIdtfyMode == Idtfy_yaw)
    {
        for(i=0; i<Sample_Num; i++)
        {
            A_Data[Vector_Order*i + 0] = identify_data[i].Acc;
            A_Data[Vector_Order*i + 1] = identify_data[i].Sign_Spd;
            y_Data[i] = identify_data[i].Input;
        }

        mat_init(&A,Sample_Num,Vector_Order,A_Data);
        mat_init(&y,Sample_Num,1,y_Data);
        mat_init(&AT,Vector_Order,Sample_Num,AT_Data);
        mat_init(&x,Vector_Order,1,x_Data);
        mat_init(&Err,Sample_Num,1,Err_Data);

        mat_init(&TEMP1,Vector_Order,Vector_Order,TEMP_data1);
        mat_init(&TEMP2,Vector_Order,Vector_Order,TEMP_data2);
        mat_init(&TEMP3,Vector_Order,Sample_Num,TEMP_data3);
        mat_init(&TEMP4,Sample_Num,1,TEMP_data4);

        mat_trans(&A,&AT);              //转置
        mat_mult(&AT,&A,&TEMP1);        //相乘
        mat_inv(&TEMP1,&TEMP2);         //求逆
        mat_mult(&TEMP2,&AT,&TEMP3);    //相乘
        mat_mult(&TEMP3,&y,&x);         //相乘
        mat_mult(&A,&x,&TEMP4);
        mat_sub(&TEMP4,&y,&Err);

        //计算残差
        for(i=0; i<Sample_Num; i++)
        {
            err_dot += (*(Err_Data+i))*(*(Err_Data+i));
        }

    }
    else if(stIdtfyCtrl.emIdtfyMode == Idtfy_pitch)
    {
        for(i=0; i<Sample_Num; i++)
        {
            A_Data[Vector_Order*i + 0] = identify_data[i].Acc;
            A_Data[Vector_Order*i + 1] = identify_data[i].Cos_theta;
            A_Data[Vector_Order*i + 2] = identify_data[i].Sin_theta;
            A_Data[Vector_Order*i + 3] = identify_data[i].Sign_Spd;
            y_Data[i] = identify_data[i].Input;
        }

        mat_init(&A,Sample_Num,Vector_Order,A_Data);
        mat_init(&y,Sample_Num,1,y_Data);
        mat_init(&AT,Vector_Order,Sample_Num,AT_Data);
        mat_init(&x,Vector_Order,1,x_Data);
        mat_init(&Err,Sample_Num,1,Err_Data);

        mat_init(&TEMP1,Vector_Order,Vector_Order,TEMP_data1);
        mat_init(&TEMP2,Vector_Order,Vector_Order,TEMP_data2);
        mat_init(&TEMP3,Vector_Order,Sample_Num,TEMP_data3);
        mat_init(&TEMP4,Sample_Num,1,TEMP_data4);

        mat_trans(&A,&AT);										//转置
        mat_mult(&AT,&A,&TEMP1);							//相乘
        mat_inv(&TEMP1,&TEMP2);							  //求逆
        mat_mult(&TEMP2,&AT,&TEMP3);					//相乘
        mat_mult(&TEMP3,&y,&x);						    //相乘
        mat_mult(&A,&x,&TEMP4);
        mat_sub(&TEMP4,&y,&Err);

        //计算残差
        for(i=0; i<Sample_Num; i++)
        {
            err_dot += (*(Err_Data+i))*(*(Err_Data+i));
        }
    }
}
