#ifndef __MOTOR_CONTORL_H
#define __MOTOR_CONTORL_H

#include <stdint.h>
#include "ris_protocol.h"

#define SATURATE(_IN, _MIN, _MAX) {\
 if (_IN <= _MIN)\
 _IN = _MIN;\
 else if (_IN >= _MAX)\
 _IN = _MAX;\
 } 
#define ST_UnitreeGO1_INIT(pKp,vKp,id) \
        {id,0,0,0,0,pKp,vKp,0,0,0,0}

#pragma pack(1)
typedef union
{
    int32_t     L;
    uint8_t     u8[4];
    uint16_t    u16[2];
    uint32_t    u32;
    float       F;
} COMData32;

typedef struct
{
    uint8_t    head[2];    // 包头         2Byte
    RIS_Mode_t mode;       // 电机控制模式  1Byte
    RIS_Fbk_t  fbk;        // 电机反馈数据 11Byte
    uint16_t   CRC16;      // CRC          2Byte
} MotorData_t;             // 返回数据    //16Byte

typedef struct
{
    uint8_t    head[2];    // 包头         2Byte
    RIS_Mode_t mode;       // 电机控制模式 1Byte
    RIS_Comd_t comd;       // 电机期望数据 12Byte
    uint16_t   CRC16;      // CRC          2Byte
} ControlData_t;           // 电机控制命令数据包//17byte
#pragma pack()

//***********************************************数据发送接收****************************************************//
#pragma pack(1)
                              /*USART6*/
/*接收结构体*/
typedef struct  
{
	unsigned char head[2];                            
    unsigned char id;                    	//电机ID: 0,1...,13,14 15表示向所有电机广播数据(此时无返回)
    unsigned char mode;                	    //工作模式: 0.锁定 1.FOC闭环 2.编码器校准 3.保留
    										//实际给FOC的指令力矩为：K_P*delta_Pos + K_W*delta_W + T
    float T;                            	//期望关节的输出力矩（电机本身的力矩）（Nm）
    float W;                            	//期望关节速度（电机本身的速度）(rad/s)
    float Pos;                          	//期望关节位置（rad）
    float K_P;                          	//关节刚度系数
    float K_W;                          	//关节速度系数

	float motor_current_M2006;		        //传给2006电机的电流值
    uint32_t id_M2006;

	signed short USART6_CRC;
}USART6_RECIEVE;

//------------------------------------------------------------------//
/*发送结构体*/
typedef struct  
{
	unsigned char head[2];
    unsigned char id;             	        //电机ID: 0,1...,13,14
    int   Temp;                           	//温度
    float T;                            	//当前实际电机输出力矩
	float W;								//当前实际电机输出角速度
    float Pos;                          	//当前电机位置（主控0点修正，电机关节还是以编码器0点为准）

	float angle_M2006;				        //2006反馈角度 （度）
	float speed_M2006;				        //2006反馈角速度 （度/秒）
    uint32_t id_M2006;

	signed short USART6_CRC;
}USART6_SEND;
#pragma pack()

typedef struct
{
    //定义 发送格式化数据
    ControlData_t  motor_send_data;
    int            hex_len;             
    long long      send_time;      
    //待发送的各项数据
    unsigned short id;          
    unsigned short mode;     
    //实际给FOC的指令力矩为：K_P*delta_Pos + K_W*delta_W + T
    float          T;       
    float          W;         
    float          Pos;      
    float          K_P;   
    float          K_W;     
    COMData32      Res;              
} MOTOR_send;

typedef struct
{
    //定义 接收数据
    MotorData_t   motor_recv_data;  
    int           hex_len;                    
    long long     resv_time;          
    int           correct;                  
    //解读得出的电机数据
    unsigned char motor_id;
    unsigned char mode;
    int           Temp;
    unsigned char MError;
    float         T;
	float         W; 
    float         Pos;
	float         footForce;
}MOTOR_recv;

typedef struct
{
    uint8_t ID;       						//电机的id
    uint8_t StatusDes;						//电机状态控制指令码

    //参数
    float Des_pos;    					//位置目标值
    float Des_velt;    					//速度目标值
    float Des_T;       					//目标力矩
    float posKp;     					//位置比例系数Kp
    float veltKp;     					//速度比例系数Kd

    //反馈信息
    float FB_pos;     					//位置反馈值
    float FB_velt;    					//角速度反馈值
    float FB_T; 						//扭矩反馈值
    int   Temp; 						//温度反馈值
}ST_UnitreeGO1;


int  modify_data(MOTOR_send *motor_s);
int  extract_data(MOTOR_recv *motor_r,uint8_t* raw_data);
void SendGO1_Motor_Data(MOTOR_send *  pData,uint8_t *raw_data);
void SendGO1_Motor_Data3(MOTOR_send *pData);

#endif
