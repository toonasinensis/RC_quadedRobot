#ifndef __HIT_GLOBAL_TYPE__
#define __HIT_GLOBAL_TYPE__

#include <stdbool.h>
#include <stdint.h>
#include "stm32h743xx.h"

/****************************************���ó���****************************************/
/****************************************���ó���****************************************/
/****************************************���ó���****************************************/

#define PI 3.141592653589793f
#define PI2 6.283185307179586f
#define PISHORT 3.14f
#define RADIAN_10 0.00174532922f // PI/1800�������Ҫ���㣬�ʵ�����ȡ����
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
	
typedef unsigned char  		UCHAR8;                  /* defined for unsigned 8-bits integer variable 	    �޷���8λ���ͱ���  */
typedef signed   char  		SCHAR8;                  /* defined for signed 8-bits integer variable		    �з���8λ���ͱ���  */
typedef unsigned short 		USHORT16;                /* defined for unsigned 16-bits integer variable 	  �޷���16λ���ͱ��� */
typedef signed   short 		SSHORT16;                /* defined for signed 16-bits integer variable 		  �з���16λ���ͱ��� */
typedef unsigned int   		UINT32;                  /* defined for unsigned 32-bits integer variable 	  �޷���32λ���ͱ��� */
typedef int   				SINT32;                      /* defined for signed 32-bits integer variable 		  �з���32λ���ͱ��� */
typedef float          		FP32;                    /* single precision floating point variable (32bits) �����ȸ�������32λ���ȣ� */
typedef double         		DB64;                    /* double precision floating point variable (64bits) ˫���ȸ�������64λ���ȣ� */

typedef UCHAR8            u8;                      /* defined for unsigned 8-bits integer variable 	    �޷���8λ���ͱ���  */
typedef USHORT16          u16;                     /* defined for unsigned 16-bits integer variable 	  �޷���16λ���ͱ��� */
typedef UINT32            u32;                     /* defined for unsigned 32-bits integer variable 	  �޷���32λ���ͱ��� */

typedef float fp32;
typedef double fp64;


/****************************************���ýṹ��****************************************/
/****************************************���ýṹ��****************************************/
/****************************************���ýṹ��****************************************/



/*PID�������ṹ��*/
typedef struct
{
    FP32 fpDes;//���Ʊ���Ŀ��ֵ
    FP32 fpFB;//���Ʊ�������ֵ

    FP32 fpKp;//����ϵ��Kp
    FP32 fpKi;//����ϵ��Ki
    FP32 fpKd;//΢��ϵ��Kd

    FP32 fpUp;//�������
    FP32 fpUi;//�������
    FP32 fpUd;//΢�����

    FP32 fpE;//����ƫ��
    FP32 fpPreE;//�ϴ�ƫ��
    FP32 fpSumE;//��ƫ��
    FP32 fpU;//����PID������

    FP32 fpUMax;//PID�����������ֵ������������ʱ������ֵ
    FP32 fpEpMax;//������������ֵ
    FP32 fpEiMax;//������������ֵ
    FP32 fpEdMax;//΢����������ֵ
    FP32 fpEMin;//��������

    FP32 fpDt;//��������
} ST_PID;

/*������̽ṹ��*/
typedef struct
{
    SINT32 siRawValue;//���α�������ԭʼֵ
    SINT32 siPreRawValue;//��һ�α�������ԭʼֵ
    SINT32 siDiff;//����������ԭʼֵ�Ĳ�ֵ
    SINT32 siSumValue;//�������ۼ�ֵ
    SINT32 siGearRatio;//������������ٱ�
    SINT32 siNumber;//����������
    FP32   fpSpeed;//��������������ת�٣���λ��r/min
} ST_ENCODER;

/*������̬���ݽṹ��*/
typedef struct{
	//��������ϵ
	float pitch;//������-oz��
	float yaw;//ƫ����-oy��
	float roll;//��ת��-ox��
	float pitch_speed;
	float yaw_speed;
	float rol_speed;
	float wx;//��ת���ٶ�
	float wy;//ƫ�����ٶ�
	float wz;//�������ٶ�
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
