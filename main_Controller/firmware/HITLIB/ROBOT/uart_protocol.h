#ifndef __UART_PROTOCOL_H__
#define __UART_PROTOCOL_H__

#include "usart.h"
#include "string.h"
#include "hit_global_type.h"
#include <cstring>

#define USART_REC_LEN  			114  	//�����������ֽ��� 

#define REMOTE_TX_DATA_LEN 	114
#define REMOTE_TX_LEN       120

#define REMOTE_RX_DATA_LEN  33
#define REMOTE_RX_LEN       40

//�Ӿ���
#define VISION_TX_DATA_LEN 	16
#define VISION_TX_LEN       21

#define VISION_RX_DATA_LEN  12
#define VISION_RX_LEN       18

typedef enum
{
	RX_FREE,
	RX_START_1,
	RX_START_2,
	RX_DATAS,
	RX_TAIL_1,
	RX_TAIL_2
}rx_protocol_e;

union float2char
{
	uint8_t char_num[4];
	fp32 float_num;	
};

typedef enum
{
	REMOTE_CTRL,
	ROBOT
}wifi_rx_id_e;

typedef struct
{
	uint8_t start1;  //֡ͷ1
	uint8_t start2;  //֡ͷ2
	
	wifi_rx_id_e ID;
	
	uint8_t datanum;
	
	uint8_t num[REMOTE_RX_DATA_LEN]; //����������
	
	uint8_t tail1;   //֡β1
	uint8_t tail2;   //֡β2
	
}uart4_rx_protocol_t;//������Ϣ���Ľṹ��//ң����

typedef struct
{
	uint8_t start1;  //֡ͷ1
	uint8_t start2;  //֡ͷ2
	
	wifi_rx_id_e ID;
	
	uint8_t datanum;
	
	uint8_t num[REMOTE_TX_DATA_LEN]; //����������
	
	uint8_t tail1;   //֡β1
	uint8_t tail2;   //֡β2
	
}uart4_tx_protocol_t;//������Ϣ���Ľṹ��//ң����



//�Ӿ�ͨѶ�ṹ��
typedef struct
{
	uint8_t start1;  //֡ͷ1
	uint8_t start2;  //֡ͷ2
		
	uint8_t datanum;
	
	uint8_t num[VISION_RX_DATA_LEN]; //����������
	
	uint8_t tail1;   //֡β1
	uint8_t tail2;   //֡β2
	
}uart5_rx_protocol_t;//������Ϣ���Ľṹ��//ң����

typedef struct
{
	uint8_t start1;  //֡ͷ1
	uint8_t start2;  //֡ͷ2
		
	uint8_t datanum;
	
	uint8_t num[VISION_TX_DATA_LEN]; //����������
	
	uint8_t tail1;   //֡β1
	uint8_t tail2;   //֡β2
	
}uart5_tx_protocol_t;//������Ϣ���Ľṹ��//ң����

typedef struct
{
   fp32 x;
	 fp32 y;
	 fp32 z;
	 fp32 pre_x;
	 fp32 pre_y;
	 fp32 pre_z;
	 fp32 x_out;
	 fp32 y_out;
	 fp32 z_out;
	 fp32 alpha_out;
	 fp32 beta_out;	
	 
}VISION_DATA;


//extern void USART1_DMA_Tx(void);
extern void Comm4Rx_IRQ(void);
extern void Comm5Rx_IRQ(void);

extern uart4_tx_protocol_t usart4_eft;
extern uart4_rx_protocol_t usart4_efr;
extern uint8_t usart4_efr_dma[REMOTE_RX_LEN];
extern uint8_t uart4_tx_data_buf[REMOTE_TX_LEN];
extern union float2char uart4_tx_buf[40];

extern uint8_t remote_rx_flag;

extern uart5_tx_protocol_t usart5_eft;
extern uart5_rx_protocol_t usart5_efr;
extern uint8_t usart5_efr_dma[VISION_RX_LEN];
extern uint8_t uart5_tx_data_buf[VISION_TX_LEN];
extern union float2char uart5_tx_buf[4];
extern VISION_DATA vision_data;


#endif
