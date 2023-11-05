#ifndef __GLOBAL_DECLARE_H__
#define __GLOBAL_DECLARE_H__
#include "hitcrt_types.h"
#include "system_monitor.h"
#include "unitree_motor.h"


/**********************************************************************************************************************
                                         身体尺寸
***********************************************************************************************************************/
#define BODY_LENGTH 0.48F
#define BODY_WIDTH 0.2425F
#define BODY_HEIGHT 0.0F

#define LEG_NUM 4 
#define JOINT_NUM 2

/**********************************************************************************************************************
                                        腿连杆长度
***********************************************************************************************************************/
#define LEG_LENGTH_0 0.0f
#define LEG_LENGTH_1 0.208f // 0.12f//0.1546f//0.16F//0.20F
#define LEG_LENGTH_2 0.282f   // 0.275f//0.40f//0.45F
#define LEG_LENGTH_3 0.0f

/***********************************************************************************************************************
                                        足端轨迹控制
************************************************************************************************************************/
#define GAIT_DT 0.002f  //控周期//0.005f
#define GAIT_MAX_S 0.23f //最大步长限幅
#define GAIT_MAX_H 0.32f

#define H_INI 0.25f // 0.10f//0.25f//0.30f//坐下腿高度
#define X_INI 0.0f  // 0.22f//0.22f//0.30f//坐下腿水平位置

//平地
#define GAIT_T 0.133f // 0.50f//步态周期
#define GAIT_S 0.04f // 0.155f//步长//这个参数没有
#define GAIT_H 0.32f // 0.22f//平地行走最大腿高
#define GAIT_h 0.272f // 0.17f//平地行走最小腿高
#define GAIT_X 0.0f // 平地行走x初值
////测试用的
//#define WALK_T 0.16f   // 1.0f(42-45s)//1.25f//3.0f
//#define WALK_H 0.31f   // 0.18f//0.165f//平地行走最大腿高
//#define WALK_h 0.264f//0.235f//0.228f  // 0.15f//0.14f//平地行走最小腿高


#define WALK_T 0.45f   // 1.0f(42-45s)//1.25f//3.0f
#define WALK_H 0.31f   // 0.18f//0.165f//平地行走最大腿高
#define WALK_h 0.274f//0.235f//0.228f  // 0.15f//0.14f//平地行走最小腿高

#define WALK_H2 0.17f  // 0.165f//0.184f//平地行走最大腿高
#define WALK_h2 0.145f // 0.14f//0.162f//平地行走最小腿高

//过绳子
#define ROPE_T 0.8f  //过绳子周期
#define ROPE_S 0.10f //过绳子步长
#define ROPE_H 0.35F //过绳子腿最大高度
#define ROPE_h 0.28f //过绳子最小腿长度

// walk上台阶
#define WALK_STAIR_T 4.5f
#define WALK_STAIR_H 0.24f
#define WALK_STAIR_h 0.12f
#define WALK_STAIR_S 0.3f

/*------------------<弹簧腿刚度阻尼>----------------------------*/
#define SpringFoot_TS 0.001f //控制周期[s]


/*************************************************************************************************************************
                                            电机和腿关节接口总宏定义

**************************************************************************************************************************/
/*-----------<两个大腿初始角度>-----------------*/

#define FOOT_Q1_INI_F  -12.0f//57.5f //-270.0f//-285.0f
#define FOOT_Q2_INI_F  12.0f//18.5f   // 0.0f//48.5f
#define FOOT_Q1_INI_H  -12.0f//57.5f //-270.0f//-285.0f
#define FOOT_Q2_INI_H  12.0f//18.5f   // 0.0f//48.5f

//目前决定 四足机器人前进方向为准，右上为腿0，顺时针旋转，分别为腿2、3、4
//大腿ID为2 解算角度为Q1， 小腿ID为1 解算角度为Q2

/*---------------<右前腿>--------------------*/
#define RF_Q1_FB stPosWheel1.fpFB
#define RF_Q1_DES stPosWheel1.fpDes
#define RF_CURRENT1_U stVeltWheel1.fpU
#define RF_CURRENT1_FW FeedForwordCur1

#define RF_Q2_FB stPosWheel2.fpFB
#define RF_Q2_DES stPosWheel2.fpDes
#define RF_CURRENT2_U stVeltWheel2.fpU
#define RF_CURRENT2_FW FeedForwordCur2

//上电初始角度
#define POS_INI_1 FOOT_Q1_INI_F
#define POS_INI_2 FOOT_Q2_INI_F

/*---------------<右后腿>--------------------*/
#define RH_Q1_FB stPosWheel3.fpFB
#define RH_Q1_DES stPosWheel3.fpDes
#define RH_CURRENT1_U stVeltWheel3.fpU
#define RH_CURRENT1_FW FeedForwordCur3

#define RH_Q2_FB stPosWheel4.fpFB
#define RH_Q2_DES stPosWheel4.fpDes
#define RH_CURRENT2_U stVeltWheel4.fpU
#define RH_CURRENT2_FW FeedForwordCur4

//上电初始角度
#define POS_INI_3 FOOT_Q1_INI_H
#define POS_INI_4 FOOT_Q2_INI_H

/*---------------<左后腿>--------------------*/
#define LH_Q1_FB stPosWheel5.fpFB
#define LH_Q1_DES stPosWheel5.fpDes
#define LH_CURRENT1_U stVeltWheel5.fpU
#define LH_CURRENT1_FW FeedForwordCur5

#define LH_Q2_FB stPosWheel6.fpFB
#define LH_Q2_DES stPosWheel6.fpDes
#define LH_CURRENT2_U stVeltWheel6.fpU
#define LH_CURRENT2_FW FeedForwordCur6

//上电初始角度
#define POS_INI_5 FOOT_Q1_INI_H
#define POS_INI_6 FOOT_Q2_INI_H

/*---------------<左前腿>--------------------*/
#define LF_Q1_FB stPosWheel7.fpFB
#define LF_Q1_DES stPosWheel7.fpDes
#define LF_CURRENT1_U stVeltWheel7.fpU
#define LF_CURRENT1_FW FeedForwordCur7

#define LF_Q2_FB stPosWheel8.fpFB
#define LF_Q2_DES stPosWheel8.fpDes
#define LF_CURRENT2_U stVeltWheel8.fpU
#define LF_CURRENT2_FW FeedForwordCur8

//上电初始角度
#define POS_INI_7 FOOT_Q1_INI_F
#define POS_INI_8 FOOT_Q2_INI_F

/*--------------<转向四个2006电机>----------*/
#define POS_INI_turn_1  0.0f
#define POS_INI_turn_2  0.0f
#define POS_INI_turn_3  0.0f
#define POS_INI_turn_4  0.0f



/********************************************************************************************************************************************
                                                 以导航相关
*********************************************************************************************************************************************/



#endif /* __GLOBAL_DECLARE_H__ */

