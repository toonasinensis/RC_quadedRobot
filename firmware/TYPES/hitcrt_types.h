/*******************************************************************
��Ȩ������HITCRT(�����󾺼�������Э��)
�ļ�����HITCRT_TYPES.h
����޸����ڣ�2023.05.24
�汾��2.0
--------------------------------------------------------------------
ģ����������ģ���������õ�����������ͨ�õĽṹ�塣
--------------------------------------------------------------------
�޸ļ�¼��
����        ʱ��            �汾         ˵��
��ΰ        2010.3.5          1.0        �������ļ�
·����      2023.5.24       2.0
********************************************************************/
#ifndef  __HITCRT_TYPES_H__
#define	 __HITCRT_TYPES_H__
#include <stdbool.h>
#include <stdint.h>
//#include "stm32f4xx_it.h"
#include <math.h>
#include "sys.h"




/****************************************���ó���****************************************/
/****************************************���ó���****************************************/
/****************************************���ó���****************************************/

#define PI 3.1415926536f
#define PI2 6.2831853072f
#define PISHORT 3.14f
#define RADIAN_10 0.00174532922f // PI/1800�������Ҫ���㣬�ʵ�����ȡ����
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


/****************************************������������****************************************/
/****************************************������������****************************************/
/****************************************������������****************************************/

#define TRUE		(1)
#define FALSE		(0)

#define DEC			(10)
#define HEX			(16)


typedef unsigned char  		UCHAR8;                    /* defined for unsigned 8-bits integer variable 	    �޷���8λ���ͱ���  */
typedef signed   char  		SCHAR8;                    /* defined for signed 8-bits integer variable		�з���8λ���ͱ���  */
typedef unsigned short 		USHORT16;                  /* defined for unsigned 16-bits integer variable 	�޷���16λ���ͱ��� */
typedef signed   short 		SSHORT16;                  /* defined for signed 16-bits integer variable 		�з���16λ���ͱ��� */
typedef unsigned int   		UINT32;                    /* defined for unsigned 32-bits integer variable 	�޷���32λ���ͱ��� */
typedef int   				    SINT32;                    /* defined for signed 32-bits integer variable 		�з���32λ���ͱ��� */
typedef float          		FP32;                      /* single precision floating point variable (32bits) �����ȸ�������32λ���ȣ� */
typedef double         		DB64;                      /* double precision floating point variable (64bits) ˫���ȸ�������64λ���ȣ� */

typedef FP32              fp32;

union float2char
{
	uint8_t char_num[4];
	fp32 float_num;	
};


/****************************************���ýṹ��****************************************/
/****************************************���ýṹ��****************************************/
/****************************************���ýṹ��****************************************/

//IMU
typedef struct
{
    FP32 pos; //��λ��
    FP32 omg; //��Ӧ����ٶ�
} ST_IMU_DIV_DATA;

typedef union {
    ST_IMU_DIV_DATA st_imu_div_data;
    SSHORT16 data[4];
} UN_IMU_DIV_DATA;

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

//����״̬ö��
typedef enum
{
    NAV_MANUAL, //�ֶ�����
    NAV_AUTO    //�ֶ�����
} EN_NAV_STATE;



/****************************************��������˳��ýṹ��****************************************/
/****************************************��������˳��ýṹ��****************************************/
/****************************************��������˳��ýṹ��****************************************/

//���غ�PID����ö��
typedef enum
{
    PID_LOOP,   // pid����ģʽ
    FORCE_CTRL, //������ģʽ
    STOP        //��ͣ
} EN_WALK_CTRL_STATE;


/*�����˵����������ṹ��*/
typedef struct
{

    float x; //�Ų�����ϵ��
    float x_pre;
    float vx;

    float y;
    float y_pre;
    float vy;

    float r; //�����꼫��
    float r_pre;
    float vr;

    float q; //����
    float theta1;
    float theta2;

    bool stance_flag; //���ر�־λ  ������1

} ST_FOOT_POS;

typedef struct
{
    //������ϵ���������ṹ��
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

//�����˵��ȵ���ǶȽṹ��
typedef struct
{
    float theta1;
    float theta2;
} ST_MOTOR_ANGLE;

//�����������˶���̬���ƽṹ��
typedef struct
{
    /*���������WALK��̬*/
    float dt;      //����������ִ������
    float T;       //һ����̬����s
    float S;       //����m
    float H;       // H=�Ų���������ߵ����m
    float h;       // h=�Ų���������͵����m
    float A;       //���洩͸���m�����ڲ������洹ֱ�����Ŀ��� AԽ�����֧����Խ��
    float phai_LF; //��ǰ����λ��
    float phai_RF; //��ǰ����λ��
    float phai_LH; //�������λ��
    float phai_RH; //�Һ�����λ��
} ST_WALK;

typedef struct
{
    /*���������TORT��̬*/
    float dt; //����������ִ������
    float T;  //һ����̬����s
    float S;  //����m
    float H;  // H=�Ų���������ߵ����m
    float h;  // h=�Ų���������͵����m
    float A;  //���洩�����m
    //����˳��LF-RH-RF-LH,���ȶ���̬
    float phai_LF; //��ǰ����λ��[0.0f,1.0f]
    float phai_RF; //��ǰ����λ��
    float phai_LH; //�������λ��
    float phai_RH; //�Һ�����λ��
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

/**����������״̬ö��**/
typedef enum
{
    READY,        //����     0
    STAND_UP,     //վ����   1
    WALK_START,   //ǰ��     2
    WALK_STOP,    //ͣ����   3
    WALK_UPSLOPE, //����     4
    JUMP,         //��Ծ     5 
    BACK_FLIP,    //��շ�   6
    FRONT_JUMP,   //         7
    SIT_DOWN, //���£�       8
    GET_DOWN, //ſ��         9
    FREEZE,   //������ǰλ�� 10
    ALL_STOP,  //��ͣ        11
	  TURN      //ԭ��ת��     12
} EN_ROBOT_STATE;

/****************************************�����PID���ýṹ��****************************************/
/****************************************�����PID���ýṹ��****************************************/
/****************************************�����PID���ýṹ��****************************************/

//���״̬��أ��������¶ȣ�
typedef struct
{
    FP32 current;     //����A
    FP32 temperature; //�¶�degree
    FP32 speed;
    FP32 position;
} ST_MOTOR_STATE;


/*������̽ṹ��*/
typedef struct
{
    SINT32 siRawValue;    //���α�������ԭʼֵ
    SINT32 siPreRawValue; //��һ�α�������ԭʼֵ
    SINT32 siDiff;        //����������ԭʼֵ�Ĳ�ֵ
    SINT32 siSumValue;    //�������ۼ�ֵ
    FP32 siGearRatio;     //������������ٱ�
    SINT32 siNumber;      //����������
    FP32 fpSpeed;         //��������������ת�٣���λ��r/min
} ST_ENCODER;

/*-----------------------------
*PID��ز���
-----------------------------*/
/*PID�ṹ��*/
typedef struct
{
    FP32 fpDes; //���Ʊ���Ŀ��ֵ        1
    FP32 fpFB;  //���Ʊ�������ֵ       2

    FP32 fpKp; //����ϵ��Kp          3
    FP32 fpKi; //����ϵ��Ki        4
    FP32 fpKd; //΢��ϵ��Kd      5

    FP32 fpE;    //����ƫ��       6
    FP32 fpPreE; //�ϴ�ƫ��      7

    FP32 fpSumE; //��ƫ��           8

    FP32 fpU;    //����PID������       9
    FP32 fpUMax; // PID�����������ֵ������������ʱ������ֵ     10
    FP32 fpEMax; //�����ַ�������ʱƫ������ֵ                  11
    FP32 fpEMin; //ƫ������                                         12

    //�����ͨ�˲���
    FP32 fpTs;     // PID��������s
    FP32 fpT;      //�˲����˲����������� G(s)=1/(T*s+1)��Խ���˲�Խ��
    FP32 fpUKp;    //���������
    FP32 fpUKi;    //���������
    FP32 fpUKd;    //΢�������
    FP32 fpUKdpre; //�ϴε�΢�������

} ST_PID;

typedef struct
{
    FP32 fpKp; //����ϵ��Kp      1
    FP32 fpKd; //΢��ϵ��Kd      2
    FP32 Ts;   //��������         3

    FP32 fpDes;   //���Ʊ���Ŀ��ֵ        3
    FP32 fpFB;    //���Ʊ�������ֵ       4
    FP32 fpFBPre; //���Ʊ�������ֵ       5

    FP32 fpU;    //����PID������       6
    FP32 fpUMax; // PID�����������ֵ������������ʱ������ֵ     7
} ST_IMPEDANCE;


//TD�������ṹ��
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
