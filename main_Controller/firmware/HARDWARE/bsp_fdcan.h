#ifndef __BSP_FDCAN_H__
#define __BSP_FDCAN_H__

#include "bsp.h"
#include "fdcan.h"

/* ֡���� */
typedef enum frame_type
{
    DATA_FRAME   = 0,  /* ����֡ */
    REMOTE_FRAME = 1,  /* Զ��֡ */
    
}_FRAME_TYPE;


/* ����CAN���Ľṹ�� */
typedef struct canmsg
{
	__IO uint16_t     id;         /* CANID */
    _FRAME_TYPE       rtr;        /* Զ��֡������֡ */
    __IO uint8_t      len;        /* CAN���ĳ��� */
    __IO uint8_t      buffer[8];  /* CAN�������� */

}_CANMSG;

/* FACAN����ṹ�� */
typedef struct fdcan_handler
{
	/* ��Ա */
	_CANMSG rx_MSG;                   /* CAN���Ľṹ�� */
	__IO uint32_t RXFIFO_Inter_Type;  /* RXFIFO�жϵ����ͣ������ݣ�ˮӡ��FIFO���) */
	__IO uint32_t RXFxS;              /* FDCAN_RXF0C �� FDCAN_RXF1C */
	__IO uint8_t  FxGI;               /* FDCAN_F0GI �� FDCAN_F1GI */
	__IO uint8_t  FxFL;               /* FDCAN_F0FL �� FDCAN_F1FL */
	__IO uint8_t  FxPI;               /* FDCAN_F0PI �� FDCAN_F1PI */

	/* ���� */
    void    (*start)(void);
    void    (*rx_Filter_Init)(void);
    void    (*rx_Interrupt_Init)(void);
    uint8_t (*send_MSG)(_CANMSG*);
    void    (*update_RXFIFO_Status)(FDCAN_HandleTypeDef *,struct fdcan_handler *);
    
}_FDCAN_HANDLER;




void FDCAN1_RX_Filter0_Init(void);
void FDCAN2_RX_Filter0_Init(void);
void FDCAN1_Interrupt_Init(void);
void FDCAN2_Interrupt_Init(void);
uint8_t FDCAN1_Send_Msg(_CANMSG *msg);
uint8_t FDCAN2_Send_Msg(_CANMSG *msg);
void FDCAN1_Start(void);
void FDCAN2_Start(void);
uint8_t look_FIFOSR_TXEFS_Bit_EFF(FDCAN_HandleTypeDef *hdfcan);
void update_RXFIFO_Status(FDCAN_HandleTypeDef *hfdcan,struct fdcan_handler *fdcan);

/* �ⲿ���� */
extern _FDCAN_HANDLER fdcan1;
extern _FDCAN_HANDLER fdcan2;
extern FDCAN_RxHeaderTypeDef hfdcan1_RX;        
extern FDCAN_RxHeaderTypeDef hfdcan2_RX; 
extern FDCAN_TxHeaderTypeDef hfdcan1_TX;        
extern FDCAN_TxHeaderTypeDef hfdcan2_TX;   
extern float degreeA, degreeB;
extern _CANMSG can1_msg;
extern _CANMSG can2_msg;
extern uint8_t dm_rx_data[3][8];

#endif /*__BSP_FDCAN_H */






