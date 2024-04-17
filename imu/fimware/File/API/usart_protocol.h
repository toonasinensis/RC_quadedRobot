#ifndef __USART_PROTOCOL_H__
#define __USART_PROTOCOL_H__


#include "stm32f4xx.h"
#include "global_declare.h"


typedef enum
{
    Vision_Type_Mask = 0x03,
    Target_Type_Mask = 0x10,
    Vision_Cmd_Mask  = 0x80,
} USART_Protocol_ID_Mask;

//协议ID号注册
typedef enum
{
    //对应掩码Vision_Type_Mask = 0x03
    Vision_Assist_Aim_Normal        = 0x00, //辅瞄-敌方正常模式
    Vision_Assist_Aim_TOP           = 0x01, //辅瞄-敌方小陀螺模式
    Vision_Interfere_BigBuff        = 0x02, //反大符
    Vision_BigBuff                  = 0x03, //大幅

    //对应掩码Target_Type_Mask = 0x10
    Vision_Target_Red               = 0x00, //目标为红
    Vision_Target_Blue              = 0x10, //目标为蓝

    //对应掩码Vision_Cmd_Mask  = 0x80
    Vision_Disable                  = 0x00, //视觉失能
    Vision_Enable                   = 0x80, //视觉使能
} USART_Protocol_ID;


typedef enum
{    
    UA1RxDMAbuf_LEN = 6,
    UA1TxDMAbuf_LEN = 6,
    UA2RxDMAbuf_LEN = 36,
    UA2TxDMAbuf_LEN = 6,
    UA3RxDMAbuf_LEN = 36,
    UA3TxDMAbuf_LEN = 36,
    UA4RxDMAbuf_LEN = 6,
    UA4TxDMAbuf_LEN = 44,
    UA6RxDMAbuf_LEN = 6,
    UA6TxDMAbuf_LEN = 36,
} USART_Data_Len;


//小陀螺数据处理结构体
typedef struct
{
//    struct
//    {
        u8 head[2];                    //2
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

        u8 tail[2];                    //2
	
//    } Send;                            //total:44

//    struct
//    {
//        u8 head[2];                 //2
//        u8 ReloadStatus;            //1
//        u8 Restart;                 //1
//        u8 tail[2];                 //2
//    } Receive;                      //total:8
} ST_IMU;

extern int Catch_Cnt ;

float Angle_Inf_To_180(float angle);
void MainControl_Rx_Protocol(void);     //串口3接收处理：主控>>>陀螺仪
void MainControl_Tx_Protocol(void);     //串口3发送处理：陀螺仪>>>主控
void Vision_Rx_Protocol(void);          //串口4接收处理：电脑>>>云台控制板
void Vision_Tx_Protocol(void);          //串口4发送处理：云台控制板>>>电脑
/********************************2024.01.24加，用于视觉组标定的协议***********************************************/
#define  USART2_TX_DATA_LEN  50
#define  USART2_RX_DATA_LEN  40
typedef struct
{
	uint8_t start1;	//帧头1
	uint8_t start2;	//帧头2
	uint8_t  id;    //id号
	uint8_t datanum;  //数组大小
	
	s8 num[USART2_TX_DATA_LEN];		//待发送数据
	
	uint8_t tail1;	//帧尾1
	uint8_t tail2;	//帧尾2
	
}usart2_tx_protocol_t;//串口2发送信息报文结构体

typedef struct
{
	uint8_t start1;  //帧头1
	uint8_t start2;  //帧头2
	uint8_t  id;    //id号
	uint8_t datanum;
	
	s8 num[USART2_RX_DATA_LEN];	//待接收数据
	
	uint8_t tail1;   //帧尾1
	uint8_t tail2;   //帧尾2
	
}usart2_rx_protocol_t;//串口2接收信息报文结构体

void Camera_Tx_Protocol(void);
extern usart2_tx_protocol_t usart2_eft;
extern usart2_rx_protocol_t usart2_efr;
extern u8 UA1RxDMAbuf[];
extern u8 UA2RxDMAbuf[];
extern u8 UA3RxDMAbuf[];
extern u8 UA4RxDMAbuf[];
extern u8 UA6RxDMAbuf[];
extern ST_IMU G_ST_IMU;
extern u8 RCStatus_Last;

extern bool Start_Catch_Flag ;
//extern ST_VISION G_ST_Vision;

#endif
