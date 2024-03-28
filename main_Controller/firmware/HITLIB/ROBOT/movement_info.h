#ifndef __MOVEMENT_INFO_H__
#define __MOVEMENT_INFO_H__

#include "hit_global_type.h"
#include "math.h"

/*����궨�ṹ��*/
typedef struct
{
	fp32 X;  //������X����λ��mm��
	fp32 Y;  //������Y����λ��mm��
	fp32 Q;  //�����Q����λ��0.1�ȣ�
}ST_POS;

typedef enum{
    CARTESIAN,//�ѿ�������ϵ
    POLAR    //������ϵ
}COORDINATE;

/*�����йؽṹ��*/
typedef struct
{
	fp32 fpVx;     //X�����
	fp32 fpVy;	   //Y�����
	fp32 fpW;      //��ת�ٶ�
	fp32 fpLength; //�������ȣ���λmm��
	fp32 fpthetha;  //������Y��Ƕȣ���λ:���ȣ�
	COORDINATE type;  //������Y��Ƕȣ���λ:���ȣ�
}ST_VECTOR;

/*������̬�ṹ��*/
typedef struct
{
	fp32 fpPosX;  //������X����λ��mm��
	fp32 fpPosY;  //������Y����λ��mm��
	fp32 fpPosQ;  //�����Q����λ��0.1�ȣ�
	fp32 fpPosX1;
	fp32 fpPosY1;
	fp32 fpPosQ1;
}ST_POT;

/*�ٶȽṹ��*/
typedef struct
{
	fp32 fpVx; //�ط����ٶȣ���λmm/s��
	fp32 fpVy; //Y�����ٶȣ���λ��mm/s��
	fp32 fpW;  //���ٶȣ���λ0.1��/s��
}ST_VELT;

typedef struct									  
{
	ST_POT 			stPot;  	 	//����������������̬
	ST_POT 			stPotPre; 		//�������ϴ�����������̬
	ST_POT 			stPotFeed;		//������ǰ������������
	ST_VELT 		stVelt;	 		//������������ȫ������ϵ�µ��ٶ�
}ST_ROBOT;

typedef struct
{
	fp32 fpQx;    //�����װ�Ƕ���ˮƽ����ļнǣ���λ��0.1�ȣ�����ֵ��0��
	fp32 fpQy;    //�����װ�Ƕ�����ֱ����ļнǣ���λ��0.1�ȣ�����ֵ��0��
	fp32 fpQz;    //�������ϵY�������������ǰ����ļнǣ���λ��0.1�ȣ�����ֵ��0��
	fp32 fpPosX0; //�����������ڻ��������ĵ�X����
	fp32 fpPosY0; //�����������ڻ��������ĵ�Y����
	fp32 fpPosZ0; //�����������ڻ��������ĵ�Z����
	fp32 fpPosX1; //Ŀ��������������ϵ��X����
	fp32 fpPosY1; //Ŀ��������������ϵ��Y����
	fp32 fpPosZ1; //Ŀ��������������ϵ��Z����
	fp32 fpPosX2; //Ŀ�������ȫ������ϵ��X����
	fp32 fpPosY2; //Ŀ�������ȫ������ϵ��Y����
	fp32 fpPosZ2; //Ŀ�������ȫ������ϵ��Z����
}ST_CAMERA;


extern ST_ROBOT stRobot;

#endif
