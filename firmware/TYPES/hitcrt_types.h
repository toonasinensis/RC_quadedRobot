/*******************************************************************
版权声明：HITCRT(哈工大竞技机器人协会)
文件名：HITCRT_TYPES.h
最近修改日期：2023.05.24
版本：2.0
--------------------------------------------------------------------
模块描述：该模块申明常用的数据类型与通用的结构体。
--------------------------------------------------------------------
修改记录：
作者        时间            版本         说明
任伟        2010.3.5          1.0        建立此文件
路方正      2023.5.24       2.0
********************************************************************/
#ifndef  __HITCRT_TYPES_H__
#define	 __HITCRT_TYPES_H__
#include <stdbool.h>
#include <stdint.h>
//#include "stm32f4xx_it.h"
#include <math.h>
#include "sys.h"




/****************************************常用常数****************************************/
/****************************************常用常数****************************************/
/****************************************常用常数****************************************/

#define PI 3.1415926536f
#define PI2 6.2831853072f
#define PISHORT 3.14f
#define RADIAN_10 0.00174532922f // PI/1800，多次需要运算，故单独提取出来
#define RADIAN_100 0.000174532922f
#define DEG 57.29578f
#define DEG_10 572.9578f     // DEG*10
#define RADIAN 0.0174532922f // PI/180
#define RAD10 572.9578f
#define PI_2 1.570796326795f
#define PI_3 1.0471975512f
#define PI_4 0.785398163f
#define PI_6 0.523598775f
#define RADIAN_15 0.261799387f
#define RADIAN_45 0.785398163f
#define RADIAN_75 1.308996939f
#define RADIAN_105 1.832595715f
#define RADIAN_135 2.356194490f
#define RADIAN_165 2.879793266f


/****************************************常用数据类型****************************************/
/****************************************常用数据类型****************************************/
/****************************************常用数据类型****************************************/

#define TRUE		(1)
#define FALSE		(0)

#define DEC			(10)
#define HEX			(16)


typedef unsigned char  		UCHAR8;                    /* defined for unsigned 8-bits integer variable 	    无符号8位整型变量  */
typedef signed   char  		SCHAR8;                    /* defined for signed 8-bits integer variable		有符号8位整型变量  */
typedef unsigned short 		USHORT16;                  /* defined for unsigned 16-bits integer variable 	无符号16位整型变量 */
typedef signed   short 		SSHORT16;                  /* defined for signed 16-bits integer variable 		有符号16位整型变量 */
typedef unsigned int   		UINT32;                    /* defined for unsigned 32-bits integer variable 	无符号32位整型变量 */
typedef int   				    SINT32;                    /* defined for signed 32-bits integer variable 		有符号32位整型变量 */
typedef float          		FP32;                      /* single precision floating point variable (32bits) 单精度浮点数（32位长度） */
typedef double         		DB64;                      /* double precision floating point variable (64bits) 双精度浮点数（64位长度） */

typedef FP32              fp32;

union float2char
{
	uint8_t char_num[4];
	fp32 float_num;	
};


/****************************************常用结构体****************************************/
/****************************************常用结构体****************************************/
/****************************************常用结构体****************************************/

//IMU
typedef struct
{
    FP32 pos; //角位置
    FP32 omg; //对应轴角速度
} ST_IMU_DIV_DATA;

typedef union {
    ST_IMU_DIV_DATA st_imu_div_data;
    SSHORT16 data[4];
} UN_IMU_DIV_DATA;

/*陀螺姿态数据结构体*/
typedef struct{
	//右手坐标系
	float pitch;//俯仰角-oz轴
	float yaw;//偏航角-oy轴
	float roll;//滚转角-ox轴
	float pitch_speed;
	float yaw_speed;
	float rol_speed;
	float wx;//滚转角速度
	float wy;//偏航角速度
	float wz;//俯仰角速度
}ST_ORIENTATION;

//导航状态枚举
typedef enum
{
    NAV_MANUAL, //手动导航
    NAV_AUTO    //手动导航
} EN_NAV_STATE;



/****************************************四足机器人常用结构体****************************************/
/****************************************四足机器人常用结构体****************************************/
/****************************************四足机器人常用结构体****************************************/

//力控和PID开关枚举
typedef enum
{
    PID_LOOP,   // pid控制模式
    FORCE_CTRL, //力控制模式
    STOP        //急停
} EN_WALK_CTRL_STATE;


/*机器人单腿足端坐标结构体*/
typedef struct
{

    float x; //髋部坐标系下
    float x_pre;
    float vx;

    float y;
    float y_pre;
    float vy;

    float r; //极坐标极径
    float r_pre;
    float vr;

    float q; //极角
    float theta1;
    float theta2;

    bool stance_flag; //触地标志位  触地置1

} ST_FOOT_POS;

typedef struct
{
    //体坐标系下足端坐标结构体
    float x;
    float y;
    float z;
} ST_FOOT_END_TO_BODY;
typedef struct
{
    FP32 x_f;
    FP32 x_h;
    FP32 y_f;
    FP32 y_h;
} ST_FOOT_JUMP_POS;

//机器人单腿电机角度结构体
typedef struct
{
    float theta1;
    float theta2;
} ST_MOTOR_ANGLE;

//机器人四足运动步态控制结构体
typedef struct
{
    /*四足机器人WALK步态*/
    float dt;      //所在任务块的执行周期
    float T;       //一个步态周期s
    float S;       //步长m
    float H;       // H=髋部到单腿最高点距离m
    float h;       // h=髋部到单腿最低点距离m
    float A;       //地面穿透深度m，用于产生地面垂直反力的控制 A越大地面支反力越大
    float phai_LF; //左前腿相位差
    float phai_RF; //右前腿相位差
    float phai_LH; //左后腿相位差
    float phai_RH; //右后腿相位差
} ST_WALK;

typedef struct
{
    /*四足机器人TORT步态*/
    float dt; //所在任务块的执行周期
    float T;  //一个步态周期s
    float S;  //步长m
    float H;  // H=髋部到单腿最高点距离m
    float h;  // h=髋部到单腿最低点距离m
    float A;  //地面穿入深度m
    //迈腿顺序LF-RH-RF-LH,最稳定步态
    float phai_LF; //左前腿相位差[0.0f,1.0f]
    float phai_RF; //右前腿相位差
    float phai_LH; //左后腿相位差
    float phai_RH; //右后腿相位差
} ST_TROT;

typedef struct
{
    float x0;
    float y0;
    float y1;
    float x1;
    float xt;
    float yt;
} ST_CZ;

/**机器人运行状态枚举**/
typedef enum
{
    READY,        //就绪     0
    STAND_UP,     //站起来   1
    WALK_START,   //前进     2
    WALK_STOP,    //停下来   3
    WALK_UPSLOPE, //上坡     4
    JUMP,         //跳跃     5 
    BACK_FLIP,    //后空翻   6
    FRONT_JUMP,   //         7
    SIT_DOWN, //坐下！       8
    GET_DOWN, //趴下         9
    FREEZE,   //锁定当前位置 10
    ALL_STOP,  //急停        11
	  TURN      //原地转向     12
} EN_ROBOT_STATE;

/****************************************电机和PID常用结构体****************************************/
/****************************************电机和PID常用结构体****************************************/
/****************************************电机和PID常用结构体****************************************/

//电机状态监控（电流、温度）
typedef struct
{
    FP32 current;     //电流A
    FP32 temperature; //温度degree
    FP32 speed;
    FP32 position;
} ST_MOTOR_STATE;


/*电机码盘结构体*/
typedef struct
{
    SINT32 siRawValue;    //本次编码器的原始值
    SINT32 siPreRawValue; //上一次编码器的原始值
    SINT32 siDiff;        //编码器两次原始值的差值
    SINT32 siSumValue;    //编码器累加值
    FP32 siGearRatio;     //电机减速器减速比
    SINT32 siNumber;      //编码器线数
    FP32 fpSpeed;         //电机减速器输出轴转速，单位：r/min
} ST_ENCODER;

/*-----------------------------
*PID相关参数
-----------------------------*/
/*PID结构体*/
typedef struct
{
    FP32 fpDes; //控制变量目标值        1
    FP32 fpFB;  //控制变量反馈值       2

    FP32 fpKp; //比例系数Kp          3
    FP32 fpKi; //积分系数Ki        4
    FP32 fpKd; //微分系数Kd      5

    FP32 fpE;    //本次偏差       6
    FP32 fpPreE; //上次偏差      7

    FP32 fpSumE; //总偏差           8

    FP32 fpU;    //本次PID运算结果       9
    FP32 fpUMax; // PID运算后输出最大值及做遇限削弱时的上限值     10
    FP32 fpEMax; //做积分分离运算时偏差的最大值                  11
    FP32 fpEMin; //偏差死区                                         12

    //加入低通滤波器
    FP32 fpTs;     // PID控制周期s
    FP32 fpT;      //滤波器滤波器特征周期 G(s)=1/(T*s+1)，越大滤波越狠
    FP32 fpUKp;    //比例项输出
    FP32 fpUKi;    //积分项输出
    FP32 fpUKd;    //微分项输出
    FP32 fpUKdpre; //上次的微分项输出

} ST_PID;

typedef struct
{
    FP32 fpKp; //比例系数Kp      1
    FP32 fpKd; //微分系数Kd      2
    FP32 Ts;   //控制周期         3

    FP32 fpDes;   //控制变量目标值        3
    FP32 fpFB;    //控制变量反馈值       4
    FP32 fpFBPre; //控制变量反馈值       5

    FP32 fpU;    //本次PID运算结果       6
    FP32 fpUMax; // PID运算后输出最大值及做遇限削弱时的上限值     7
} ST_IMPEDANCE;


//TD跟踪器结构体
typedef struct
{
	float x1; //
	float x2;
	float x3;
	float x;
	float r;
	float h;
	float T;
	float aim;
} TD;



#endif
