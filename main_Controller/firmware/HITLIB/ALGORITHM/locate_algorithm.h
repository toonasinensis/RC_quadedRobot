#ifndef __LOCATE_ALGORITHM_H__
#define __LOCATE_ALGORITHM_H__

#include "hit_global_type.h"
#include "filter_algorithm.h"
#include "math_algorithm.h"
#include "stm32h7xx_it.h"
#include "bsp_fdcan.h"

#include "navigation_task.h"

//ע���޸ļ�����  ���ڼ�������˵��ٶ�
#define TIM12_BASE_TIME 0.000001 //��λs


/***************************************************��λ********************************************************/
/*ģ�����ݡ����������л�*/
#define DigtalGyro 0
#define AnologGyro 0
#define GambalGyro 1


/*�涯��ͨ������*/
#define FollowerWheel_CoderA_Ch 4 //�涯��Aͨ��
#define FollowerWheel_CoderB_Ch 3 //�涯��Bͨ��

#define ALPHA_A_Inc 2.348157513749949  //2.334947911447260f	// 0.265021312755329f
#define ALPHA_A_Dec 2.348469015344453  //2.333119413056502f	// 0.269511583664554f
#define ALPHA_B_Inc -2.342543769582928   //-2.370594424560095f // 1.821841970578082f
#define ALPHA_B_Dec -2.343236349762283   //-2.372491401540901f // 1.823686222540484f

#define FW_Len_A_Inc -0.221224200628719 //-0.446915282605278f //-0.222691289129401f
#define FW_Len_A_Dec -0.221254381324861 //-0.446158614546869f //-0.221135445952228f
#define FW_Len_B_Inc -0.223950618769521 //-0.444852899249265f //-0.221160257514818f
#define FW_Len_B_Dec -0.223699879963817 //-0.444611541604590f //-0.221135445952228f

/*�������������涯�����������Ƕ�(ȫ��Y)��ģ*/
#define FW_Rob_Len    210.3  //153.73//-79.798596253165260f	 //�涯����������������ĵľ���
#define FW_rob_Alpha -1.5707963268//-2.900048578694853f //�涯�����������������ļнǣ���λ�����ȣ�

/*������ϵ��*/
#define K_ANTICLOCK   1.0 //1.0292330784075477092416553498677721392323f // 1.0596067852746241345313233485224f//
#define K_CLOCK    1.0 //1.0297482837528604118993135011441647597254f		// 1.0590151699511370583582578864714f//

/*
DT35ϵ��
				|y+
				|dt35_y
				|
x+����������������������������������������������������dt35_x
				|
				|
				|
*/

#define K_DT35_X1 -1.112127110963998f
#define B_DT35_X1 -8.118602110400346f

#define K_DT35_X2 -1.003652393891736f
#define B_DT35_X2 -1.330248591758925e+02f

#define K_DT35_Y1 1.012553393969609f
#define B_DT35_Y1 1.238437436213362e+02f

#define K_DT35_Y2 1.043767942624321f
#define B_DT35_Y2 60.577194894797600f


//����
#define DIS_X1_DT35_TO_C1 109.34f // 293.275f
#define DIS_X1_DT35_TO_C2 172.98f // 297.25f
#define DIS_X2_DT35_TO_C1 60.96f  // 293.275f
#define DIS_X2_DT35_TO_C2 222.98f // 297.25f
#define DIS_Y1_DT35_TO_C1 158.15f // 261.0f
#define DIS_Y1_DT35_TO_C2 116.81f // 174.99f
#define DIS_Y2_DT35_TO_C1 158.15f // 110.0f
#define DIS_Y2_DT35_TO_C2 116.81f // 334.99f

// #define R_DT35_DOWN		306.47f
// #define R_DT35_RIGHT	244.16f

/*dt35����ת���ĵĸ������룬����Ϊ����*/
#define DIS_UP_DT35 (420.0f / tanf(PI / 3.0))
#define DIS_LEFT_DT35 215.0f

#define DIS_X1_DT35_TO_MID (586.55f / 2.0)
#define DIS_X2_DT35_TO_MID (586.55f / 2.0)
#define DIS_Y1_DT35_TO_MID 305.0f
#define DIS_Y2_DT35_TO_MID 65.0f

#define HIDE_Y 140.0f

 #define CAMERA_X 345.15f
#define CAMERA_Y 86.0f

/*�涯�����̹��߼������涯����ؽṹ��*/
typedef struct
{
	int32_t siCoderACur; //��ǰ����A����
	int32_t siCoderAPre; //��һ������A�������ж��涯����ת����
	int32_t siCoderBCur; //��ǰ����B����
	int32_t siCoderBPre; //��һ������B����

	ST_VECTOR stVectorFWCen_RobCen; //�涯����������������ĵ���������Ҫ��y����ԽǶȺ;��룩

	ST_POT stPot;	 //�涯������������̬
	ST_POT stPotPre; //�涯���ϴ���������
} ST_FOLLOWER_WHEEL;

/*���ݽṹ��*/
typedef struct
{
	fp32 fpClock;	  //˳ʱ��ϵ��
	fp32 fpAntiClock; //��ʱ��ϵ��
	fp32 fpQ_Cur;	  //���ݵ�ǰ���ݶ���
	fp32 fpQ_Pre;	  //������һ�����ݶ������ж���ת����
} ST_GYRO;

/*DT35��λ�ṹ��*/
//�ĸ�DT35����UP��������DT35��x������Ϊy1��y2����LEFT��������DT35��y������Ϊx1��x2
typedef struct
{
	fp32 robot_x, robot_y, robot_q;
	fp32 dt35_x1, dt35_x2, dt35_y1, dt35_y2;
	fp32 dt35_fix_x1, dt35_fix_x2, dt35_fix_y1, dt35_fix_y2;
	fp32 pro_x1, pro_x2, pro_y1, pro_y2;
	fp32 robot_x1, robot_x2, robot_y1, robot_y2;
} dt35_t;

extern void RobotLocation(ST_ROBOT *pstRobot, ST_FOLLOWER_WHEEL *pstFW, ST_GYRO *pstGyro);
extern void CalibrateRobotVelt(ST_ROBOT *pstRobot);
extern void DT35_relocation(ST_ROBOT *pstRobot, ST_FOLLOWER_WHEEL *pstFW, dt35_t *p_dt35_save, dt35_t *p_dt35_now);
void DT35_relocation_new(ST_ROBOT *pstRobot, ST_FOLLOWER_WHEEL *pstFW, dt35_t *p_dt35_save, dt35_t *p_dt35_now);
extern fp32 DT35_Q_relocation(void);

extern ST_FOLLOWER_WHEEL stFollowerWheel;
extern ST_GYRO stGyro;
extern dt35_t dt35_save, dt35_now, dt35_wrestle_save;

extern fp32 fpPosXOffset; // X�����ƫ������
extern fp32 fpPosYOffset; // Y�����ƫ������
extern fp32 fpStartX;
extern fp32 fpStartY;
extern float dt35_x_number, dt35_y_number;
extern int reloc_times;

extern float DT35_Q;
extern void DT35_Q_relocation(dt35_t *p_dt35_now);
//extern void Vision_relocation_RED(ST_ROBOT *pstRobot, ST_FOLLOWER_WHEEL *pstFW,tower *p_tower);
//extern void Vision_relocation_BLUE(ST_ROBOT *pstRobot, ST_FOLLOWER_WHEEL *pstFW,tower *p_tower);
extern void DT35_Boundary_Blue_1(ST_ROBOT *pstRobot, ST_FOLLOWER_WHEEL *pstFW, dt35_t *p_dt35_now);
extern int dt35_movemode;

#endif
