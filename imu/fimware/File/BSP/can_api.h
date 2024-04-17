#ifndef __CANAPI_H__
#define __CANAPI_H__
#include  "main.h"

#define CAN1_RX_BUF_MAX 64/*������2����*/
#define CAN1_TX_BUF_MAX 64/*������2����*/
#define CAN1_RX_BUF_MARK	(CAN1_RX_BUF_MAX-1)
#define CAN1_TX_BUF_MARK	(CAN1_TX_BUF_MAX-1)

extern bool CanCmdManage(CanRxMsg *pRxMessage, CanTxMsg *pTxMessage);

//���º���Ϊ�������
extern void CAN1PutDatatoTxBuf(CanTxMsg *pTxMessage);
extern bool CAN1IsDataInTxBuf( void );
extern UINT32 CAN1GetTxBufLen(void);
extern CanTxMsg *CAN1GetTxBufDat( void );
extern void CAN1BeginSend(void);
extern void CAN1StopSend(void);

//���º���Ϊ�������
extern void CAN1PutDatatoRxBuf(CanRxMsg *pRxMessage);
extern bool CAN1IsDataInRxBuf(void);
extern UINT32 CAN1GetRxBufLen(void);
extern CanRxMsg *CAN1GetRxBufDat( void );


#endif
