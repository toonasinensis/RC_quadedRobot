#ifndef __MOVEMENT_INFO_H__
#define __MOVEMENT_INFO_H__

#include "hit_global_type.h"
#include "math.h"

/*坐标标定结构体*/
typedef struct
{
	fp32 X;  //横坐标X（单位：mm）
	fp32 Y;  //竖坐标Y（单位：mm）
	fp32 Q;  //航向角Q（单位：0.1度）
}ST_POS;

typedef enum{
    CARTESIAN,//笛卡尔坐标系
    POLAR    //极坐标系
}COORDINATE;

/*向量有关结构体*/
typedef struct
{
	fp32 fpVx;     //X方向差
	fp32 fpVy;	   //Y方向差
	fp32 fpW;      //旋转速度
	fp32 fpLength; //向量长度（单位mm）
	fp32 fpthetha;  //向量与Y轴角度（单位:弧度）
	COORDINATE type;  //向量与Y轴角度（单位:弧度）
}ST_VECTOR;

/*坐标姿态结构体*/
typedef struct
{
	fp32 fpPosX;  //横坐标X（单位：mm）
	fp32 fpPosY;  //竖坐标Y（单位：mm）
	fp32 fpPosQ;  //航向角Q（单位：0.1度）
	fp32 fpPosX1;
	fp32 fpPosY1;
	fp32 fpPosQ1;
}ST_POT;

/*速度结构体*/
typedef struct
{
	fp32 fpVx; //Ｘ方向速度（单位mm/s）
	fp32 fpVy; //Y方向速度（单位：mm/s）
	fp32 fpW;  //角速度（单位0.1度/s）
}ST_VELT;

typedef struct									  
{
	ST_POT 			stPot;  	 	//机器人中心坐标姿态
	ST_POT 			stPotPre; 		//机器人上次中心坐标姿态
	ST_POT 			stPotFeed;		//机器人前馈运算后的坐标
	ST_VELT 		stVelt;	 		//机器人中心在全场坐标系下的速度
}ST_ROBOT;

typedef struct
{
	fp32 fpQx;    //相机安装角度与水平方向的夹角（单位：0.1度，理想值是0）
	fp32 fpQy;    //相机安装角度与竖直方向的夹角（单位：0.1度，理想值是0）
	fp32 fpQz;    //相机坐标系Y方向与机器人正前方向的夹角（单位：0.1度，理想值是0）
	fp32 fpPosX0; //相机中心相对于机器人中心的X坐标
	fp32 fpPosY0; //相机中心相对于机器人中心的Y坐标
	fp32 fpPosZ0; //相机中心相对于机器人中心的Z坐标
	fp32 fpPosX1; //目标相对于相机坐标系的X坐标
	fp32 fpPosY1; //目标相对于相机坐标系的Y坐标
	fp32 fpPosZ1; //目标相对于相机坐标系的Z坐标
	fp32 fpPosX2; //目标相对于全局坐标系的X坐标
	fp32 fpPosY2; //目标相对于全局坐标系的Y坐标
	fp32 fpPosZ2; //目标相对于全局坐标系的Z坐标
}ST_CAMERA;


extern ST_ROBOT stRobot;

#endif
