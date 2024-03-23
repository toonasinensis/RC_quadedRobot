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

//Э��ID��ע��
typedef enum
{
    //��Ӧ����Vision_Type_Mask = 0x03
    Vision_Assist_Aim_Normal        = 0x00, //����-�з�����ģʽ
    Vision_Assist_Aim_TOP           = 0x01, //����-�з�С����ģʽ
    Vision_Interfere_BigBuff        = 0x02, //�����
    Vision_BigBuff                  = 0x03, //���

    //��Ӧ����Target_Type_Mask = 0x10
    Vision_Target_Red               = 0x00, //Ŀ��Ϊ��
    Vision_Target_Blue              = 0x10, //Ŀ��Ϊ��

    //��Ӧ����Vision_Cmd_Mask  = 0x80
    Vision_Disable                  = 0x00, //�Ӿ�ʧ��
    Vision_Enable                   = 0x80, //�Ӿ�ʹ��
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


//С�������ݴ���ṹ��
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
void MainControl_Rx_Protocol(void);     //����3���մ�������>>>������
void MainControl_Tx_Protocol(void);     //����3���ʹ���������>>>����
void Vision_Rx_Protocol(void);          //����4���մ�������>>>��̨���ư�
void Vision_Tx_Protocol(void);          //����4���ʹ�����̨���ư�>>>����
/********************************2024.01.24�ӣ������Ӿ���궨��Э��***********************************************/
#define  USART2_TX_DATA_LEN  50
#define  USART2_RX_DATA_LEN  40
typedef struct
{
	uint8_t start1;	//֡ͷ1
	uint8_t start2;	//֡ͷ2
	uint8_t  id;    //id��
	uint8_t datanum;  //�����С
	
	s8 num[USART2_TX_DATA_LEN];		//����������
	
	uint8_t tail1;	//֡β1
	uint8_t tail2;	//֡β2
	
}usart2_tx_protocol_t;//����2������Ϣ���Ľṹ��

typedef struct
{
	uint8_t start1;  //֡ͷ1
	uint8_t start2;  //֡ͷ2
	uint8_t  id;    //id��
	uint8_t datanum;
	
	s8 num[USART2_RX_DATA_LEN];	//����������
	
	uint8_t tail1;   //֡β1
	uint8_t tail2;   //֡β2
	
}usart2_rx_protocol_t;//����2������Ϣ���Ľṹ��

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
