#ifndef __HIT_GLOBAL_TYPE__
#define __HIT_GLOBAL_TYPE__

#include <stdbool.h>
#include <stdint.h>
#include "stm32h743xx.h"

/****************************************常用常数****************************************/
/****************************************常用常数****************************************/
/****************************************常用常数****************************************/

#define PI 3.141592653589793f
#define PI2 6.283185307179586f
#define PISHORT 3.14f
#define RADIAN_10 0.00174532922f // PI/1800，多次需要运算，故单独提取出来
#define RADIAN_100 0.000174532922f
#define DEG 57.295779513082321f
#define DEG_10 572.9578f       // DEG*10
#define RADIAN 0.0174532925199433f   // PI/180
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


#define NULL 0 

#define FALSE false
#define TRUE true
	
typedef unsigned char  		UCHAR8;                  /* defined for unsigned 8-bits integer variable 	    无符号8位整型变量  */
typedef signed   char  		SCHAR8;                  /* defined for signed 8-bits integer variable		    有符号8位整型变量  */
typedef unsigned short 		USHORT16;                /* defined for unsigned 16-bits integer variable 	  无符号16位整型变量 */
typedef signed   short 		SSHORT16;                /* defined for signed 16-bits integer variable 		  有符号16位整型变量 */
typedef unsigned int   		UINT32;                  /* defined for unsigned 32-bits integer variable 	  无符号32位整型变量 */
typedef int   				SINT32;                      /* defined for signed 32-bits integer variable 		  有符号32位整型变量 */
typedef float          		FP32;                    /* single precision floating point variable (32bits) 单精度浮点数（32位长度） */
typedef double         		DB64;                    /* double precision floating point variable (64bits) 双精度浮点数（64位长度） */

typedef UCHAR8            u8;                      /* defined for unsigned 8-bits integer variable 	    无符号8位整型变量  */
typedef USHORT16          u16;                     /* defined for unsigned 16-bits integer variable 	  无符号16位整型变量 */
typedef UINT32            u32;                     /* defined for unsigned 32-bits integer variable 	  无符号32位整型变量 */

typedef float fp32;
typedef double fp64;


/****************************************常用结构体****************************************/
/****************************************常用结构体****************************************/
/****************************************常用结构体****************************************/



/*PID控制器结构体*/
typedef struct
{
    FP32 fpDes;//控制变量目标值
    FP32 fpFB;//控制变量反馈值

    FP32 fpKp;//比例系数Kp
    FP32 fpKi;//积分系数Ki
    FP32 fpKd;//微分系数Kd

    FP32 fpUp;//比例输出
    FP32 fpUi;//积分输出
    FP32 fpUd;//微分输出

    FP32 fpE;//本次偏差
    FP32 fpPreE;//上次偏差
    FP32 fpSumE;//总偏差
    FP32 fpU;//本次PID运算结果

    FP32 fpUMax;//PID运算后输出最大值及做遇限削弱时的上限值
    FP32 fpEpMax;//比例项输出最大值
    FP32 fpEiMax;//积分项输出最大值
    FP32 fpEdMax;//微分项输出最大值
    FP32 fpEMin;//积分上限

    FP32 fpDt;//控制周期
} ST_PID;

/*电机码盘结构体*/
typedef struct
{
    SINT32 siRawValue;//本次编码器的原始值
    SINT32 siPreRawValue;//上一次编码器的原始值
    SINT32 siDiff;//编码器两次原始值的差值
    SINT32 siSumValue;//编码器累加值
    SINT32 siGearRatio;//电机减速器减速比
    SINT32 siNumber;//编码器线数
    FP32   fpSpeed;//电机减速器输出轴转速，单位：r/min
} ST_ENCODER;

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


typedef struct
{

        uint8_t head[2];                    //2
			  float ang_vel_x;             //4
        float ang_vel_y;             //4
        float ang_vel_z;               //4

        float quat_X;              //4
        float quat_Y;              //4
        float quat_Z;              //4
			  float quat_W;              //4
			
			  float acc_x_send;             //4
        float acc_y_send;             //4
        float acc_z_send;               //4

        uint8_t tail[2];                    //2

} ST_IMU;



#endif
