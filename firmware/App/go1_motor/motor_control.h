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
    uint8_t    head[2];    // ��ͷ         2Byte
    RIS_Mode_t mode;       // �������ģʽ  1Byte
    RIS_Fbk_t  fbk;        // ����������� 11Byte
    uint16_t   CRC16;      // CRC          2Byte
} MotorData_t;             // ��������    //16Byte

typedef struct
{
    uint8_t    head[2];    // ��ͷ         2Byte
    RIS_Mode_t mode;       // �������ģʽ 1Byte
    RIS_Comd_t comd;       // ����������� 12Byte
    uint16_t   CRC16;      // CRC          2Byte
} ControlData_t;           // ��������������ݰ�//17byte
#pragma pack()

//***********************************************���ݷ��ͽ���****************************************************//
#pragma pack(1)
                              /*USART6*/
/*���սṹ��*/
typedef struct  
{
	unsigned char head[2];                            
    unsigned char id;                    	//���ID: 0,1...,13,14 15��ʾ�����е���㲥����(��ʱ�޷���)
    unsigned char mode;                	    //����ģʽ: 0.���� 1.FOC�ջ� 2.������У׼ 3.����
    										//ʵ�ʸ�FOC��ָ������Ϊ��K_P*delta_Pos + K_W*delta_W + T
    float T;                            	//�����ؽڵ�������أ������������أ���Nm��
    float W;                            	//�����ؽ��ٶȣ����������ٶȣ�(rad/s)
    float Pos;                          	//�����ؽ�λ�ã�rad��
    float K_P;                          	//�ؽڸն�ϵ��
    float K_W;                          	//�ؽ��ٶ�ϵ��

	float motor_current_M2006;		        //����2006����ĵ���ֵ
    uint32_t id_M2006;

	signed short USART6_CRC;
}USART6_RECIEVE;

//------------------------------------------------------------------//
/*���ͽṹ��*/
typedef struct  
{
	unsigned char head[2];
    unsigned char id;             	        //���ID: 0,1...,13,14
    int   Temp;                           	//�¶�
    float T;                            	//��ǰʵ�ʵ���������
	float W;								//��ǰʵ�ʵ��������ٶ�
    float Pos;                          	//��ǰ���λ�ã�����0������������ؽڻ����Ա�����0��Ϊ׼��

	float angle_M2006;				        //2006�����Ƕ� ���ȣ�
	float speed_M2006;				        //2006�������ٶ� ����/�룩
    uint32_t id_M2006;

	signed short USART6_CRC;
}USART6_SEND;
#pragma pack()

typedef struct
{
    //���� ���͸�ʽ������
    ControlData_t  motor_send_data;
    int            hex_len;             
    long long      send_time;      
    //�����͵ĸ�������
    unsigned short id;          
    unsigned short mode;     
    //ʵ�ʸ�FOC��ָ������Ϊ��K_P*delta_Pos + K_W*delta_W + T
    float          T;       
    float          W;         
    float          Pos;      
    float          K_P;   
    float          K_W;     
    COMData32      Res;              
} MOTOR_send;

typedef struct
{
    //���� ��������
    MotorData_t   motor_recv_data;  
    int           hex_len;                    
    long long     resv_time;          
    int           correct;                  
    //����ó��ĵ������
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
    uint8_t ID;       						//�����id
    uint8_t StatusDes;						//���״̬����ָ����

    //����
    float Des_pos;    					//λ��Ŀ��ֵ
    float Des_velt;    					//�ٶ�Ŀ��ֵ
    float Des_T;       					//Ŀ������
    float posKp;     					//λ�ñ���ϵ��Kp
    float veltKp;     					//�ٶȱ���ϵ��Kd

    //������Ϣ
    float FB_pos;     					//λ�÷���ֵ
    float FB_velt;    					//���ٶȷ���ֵ
    float FB_T; 						//Ť�ط���ֵ
    int   Temp; 						//�¶ȷ���ֵ
}ST_UnitreeGO1;


int  modify_data(MOTOR_send *motor_s);
int  extract_data(MOTOR_recv *motor_r,uint8_t* raw_data);
void SendGO1_Motor_Data(MOTOR_send *  pData,uint8_t *raw_data);
void SendGO1_Motor_Data3(MOTOR_send *pData);

#endif
