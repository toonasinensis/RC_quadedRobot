#ifndef __FILTER_ALGORITHM_H__
#define __FILTER_ALGORITHM_H__

#include "hit_global_type.h"

typedef struct
{
	fp32 preout;
	fp32 out;
	fp32 in;
	fp32 off_freq;//权重
	fp32 samp_tim;//采样步长
}ST_LPF;

typedef struct
{
    float raw_value;
    float filtered_value[2];
    float xhat_data[2], xhatminus_data[2], z_data[2],Pminus_data[4], K_data[4];
    float P_data[4];
    float AT_data[4], HT_data[4];
    float A_data[4];
    float H_data[4];
    float Q_data[4];
    float R_data[4];
} kalman_filter_init_t;


///******************************************************************************************
//函数名：简单卡尔曼滤波
//输入参数：
//输出参数：
//说明：Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏  R:测量噪声，R增大，动态响应变慢，收敛稳定性变好
//*********************************************************************************************/
//#define KALMAN_FILTER(output,input,ProcessNiose_Q,MeasureNoise_R) do{\
//    double R = MeasureNoise_R;\
//    double Q = ProcessNiose_Q;\
//    static double input##x_last;\
//    double x_mid = input##x_last;\
//    double x_now;\
//    static double input##p_last;\
//    double p_mid ;\
//    double p_now;\
//    double kg;\
//    x_mid=input##x_last;\
//    p_mid=input##p_last+Q;\
//    kg=p_mid/(p_mid+R);\
//    x_now=x_mid+kg*((input)-x_mid);\
//    p_now=(1.0f-kg)*p_mid;\
//    input##p_last = p_now;\
//    input##x_last = x_now;\
//    output = x_now;\
//}while(0)


extern void LpFilter(ST_LPF* lpf);

#endif
