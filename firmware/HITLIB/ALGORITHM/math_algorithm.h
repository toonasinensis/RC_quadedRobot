/*---------------------------------------------------------------------
��Ȩ������HITCRT(�����󾺼������˶�)
�ļ�����HITCRT_Coff.h
����޸����ڣ�2016.04.04
�汾��1.0
---------------------------------------------------------------------*/
#ifndef __MATH_ALGORITHM_H__
#define __MATH_ALGORITHM_H__

#include "hit_global_type.h"
#include "movement_info.h"
#include "math.h"

#define COS(a) cosf(a*RADIAN)
#define SIN(a) sinf(a*RADIAN)

#define EPS 1e-5

#define COS30		0.86602540378f
#define SIN30 		0.5f

typedef struct
{
	enum NAV_LOCK;
}ST_NAV;

/*������*/
typedef struct
{
	fp32 x;
	fp32 y;
}stPoint;

typedef struct
{
	fp32 x1;
	fp32 y1;
	fp32 x2;
	fp32 y2;
}stDT35;

typedef struct
{
	fp32 s;
	fp32 e;
}ANGLE;

typedef struct
{
	uint8_t 		n;
	uint32_t		t_sum;
	stPoint		Pi[4];
}stBezier_Para;//���������߽ṹ��

typedef struct
{
	fp32 t11;//����ʱ��
	fp32 t1;//����ʱ��
	ST_VELT Accup;//���ټ��ٶ�
	ST_VELT Accdown;//���ټ��ٶ�
	ST_VELT VIN;//��ʼ����ʱ�ĳ��ٶ�
}T_fasedown;//���ٽṹ�壬�ȼ����ټ���	

/*2021����·����*/
typedef struct
{
	fp32 x, y;
}point_t;

typedef struct
{
	fp32 A, B, C;//Ax + By + C = 0
}line_t;


extern int32_t Round(fp32 fpValue);//��������������ת��Ϊ����������
extern fp32 CalRadialProjection(ST_VECTOR stAim, ST_VECTOR stBase);//����һ�����ڻ�׼��������ͶӰ
extern fp32 CalNormalProjection(ST_VECTOR stAim,ST_VECTOR stBase);//����һ�����ڻ�׼����������ͶӰ
extern fp32 CalAngle(ST_VECTOR stAim);//����һ������ssPosY��������н�
extern fp32 ConvertAngle(fp32 fpAngA);//���Ƕ�ת��Ϊȫ������ϵ�ĺ���Ƿ�Χ[-PI,PI)(��λ������)
extern int16_t ConvertDeg(int16_t fpDegA);//���Ƕ�ת��Ϊȫ������ϵ�ĺ���Ƿ�Χ[-1800,1800)(��λ��0.1��)
extern int32_t Clip(int32_t siValue, int32_t siMin, int32_t siMax);//��������������ȥ���������ֵ����Сֵ֮���ֵ����֮��������Сֵ
extern uint8_t ClipChar(uint8_t ucValue, uint8_t ucMin, uint8_t ucMax);
extern fp32 ClipFloat(fp32 fpValue, fp32 fpMin, fp32 fpMax);//����������������ȥ���������ֵ����Сֵ֮���ֵ����֮��������Сֵ
extern fp32 Fabs(fp32 fpNum);//�������ֵ����
extern uint16_t crc16(uint8_t *pucData,uint8_t ucLength);//�Թ̶����ȵ���������CRCУ����
extern uint8_t sum8(uint8_t *data,uint8_t length);//��ͺ���
extern void us_delay(uint32_t uiTime);//��ʱus

extern stPoint tangentdot(stPoint ptCenter, stPoint ptOutside, fp32 Radious, int mode);
extern fp32 SLine(stPoint p1,stPoint p2);
extern stPoint Cirpoint(stPoint p1,stPoint p2,stPoint p3,fp32 Radius);
extern fp32 Angle(stPoint p0,stPoint p1);
extern fp32 SCircle(stPoint p1,stPoint p2,stPoint p0);
//extern uint8_t Bezier_Nav(ST_NAV *pstNav, ST_POT *Des, stBezier_Para *pstBezier, ST_VELT * pstVel);
extern fp32 CalTime_min(fp32 A1,fp32 A2,fp32 S,fp32 V);
extern fp32 CalTime_mid(fp32 A1,fp32 A2,fp32 S,fp32 V);
extern fp32 CalRushTime(fp32 A,fp32 S,fp32 V);
extern void CalLineTime(fp32 A1,fp32 A2,fp32 S,fp32 *T1,fp32 *T11);
extern void CalT_dST(T_fasedown * T_dST,stPoint Start,stPoint End,ST_VELT Vin,fp32 A);
extern void Trapezoid_Speed(fp32 A,fp32 Vmax,fp32 S,fp32 *T1,fp32 *T11,fp32 *T12);
extern int8_t sign_judge(float fp_Judge_Number);

extern int32_t my_intabs(int32_t num);
extern fp32 cal_min_angle(fp32 des, fp32 fdb);
extern void Concert_coorindnate(ST_VECTOR *global,ST_VECTOR *local,fp32 fpQ);//��ȫ������ϵת��Ϊ��������ϵ����
fp32 Geometric_mean(fp32 a,fp32 b);
void Vector_minus(ST_VECTOR *a,ST_VECTOR *b,ST_VECTOR *c);
void Vector_Plus(ST_VECTOR *a,ST_VECTOR *b,ST_VECTOR *c);
void Vector_cross(ST_VECTOR *r,fp32 w,ST_VECTOR *c);
void Covert_coordinate(ST_VECTOR *a,COORDINATE _type);
void Vector_Product(ST_VECTOR *a,ST_VECTOR *b,fp32 *c);
void Vector_Projection_BonA(ST_VECTOR *a,ST_VECTOR *b,ST_VECTOR *c);
void Vector_Projection_BonA_2(ST_VECTOR *a,ST_VECTOR *b,fp32 *c);
void Vector_Turn(ST_VECTOR *a,ST_VECTOR *b,fp32 c);


#endif
