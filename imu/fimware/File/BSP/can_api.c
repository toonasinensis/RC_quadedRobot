#include "main.h"
#include "math.h"
#include "can_api.h"
#include "flash_api.h"
#include "pmsm_control.h"
#include "pmsm_control_types.h"


static CanRxMsg CAN1_RX_Buf[CAN1_RX_BUF_MAX];		//����
static CanTxMsg CAN1_TX_Buf[CAN1_TX_BUF_MAX];		//����

volatile u32 CAN1_Rx_Head = 0;					//����ͷ-���յ�ʱ���ƶ�
volatile u32 CAN1_Rx_Tail = 0;					//����β-��ȡ��ʱ���ƶ�

volatile u32 CAN1_Tx_Head = 0;					//����ͷ-���յ�ʱ���ƶ�
volatile u32 CAN1_Tx_Tail = 0;					//����β-��ȡ��ʱ���ƶ�

/****************************************************************************************************
��������: CAN1PutDatatoTxBuf()
��������: �����ݷŽ����Ͷ�����
�������: CanTxMsg *pTxMessage
���ز���: ��
��   ע: �û���Ҫ�����ݷ���ʱ��ʹ��
****************************************************************************************************/
void CAN1PutDatatoTxBuf(CanTxMsg *pTxMessage)
{
    u32 tmphead;
    tmphead = ( CAN1_Tx_Head + 1 ) & CAN1_TX_BUF_MARK;//����ĩ���ж�,����ĩ��,����0
    CAN1_Tx_Head = tmphead; 	// ÿ����,����ͷ����1
    CAN1_TX_Buf[tmphead] = *pTxMessage;

}

/****************************************************************************************************
��������: CAN1IsDataInTxBuf()
��������: ��֪�������Ƿ�������
�������: ��
���ز���: ��
��   ע: �����е�ͷ��β����ȵ�ʱ��,�ʹ������������ڻ�����
****************************************************************************************************/
bool CAN1IsDataInTxBuf( void )
{
    if( CAN1_Tx_Head == CAN1_Tx_Tail )
        return FALSE;
    else
        return TRUE;
}

/****************************************************************************************************
��������: CAN1GetTxBufLen()
��������: ��ȡ��������Ч���ݵĳ���
�������: ��
���ز���: ��
��   ע: :��ȡ��ֵ��ȻΪ��Сֵ,��Ϊ��ʵ���Ȼ᲻�ϱ仯.������32λ��ARMϵͳ,��ȡ32λ���ݲ������ٽ�����,���Կ��Բ����ǹ��ж�
**			��ν��Ч,��ָʣ��Ŀ��ó���
****************************************************************************************************/
u32 CAN1GetTxBufLen(void)
{
    //__disalbe_irq();
    if(CAN1_Tx_Head >= CAN1_Tx_Tail)
    {
        //__enable_irq();
        return(CAN1_TX_BUF_MAX - (CAN1_Tx_Head-CAN1_Tx_Tail));
    }
    else
    {
        //__enable_irq();
        return(CAN1_Tx_Tail - CAN1_Tx_Head);
    }
}

/****************************************************************************************************
��������: CAN1GetTxBufDat()
��������: �Ӷ����л�ȡ����
�������: ��
���ز���: ��
��   ע:  ���ô˺���ǰ����ȷ��������������!!
****************************************************************************************************/
CanTxMsg *CAN1GetTxBufDat( void )
{
    u32 tmptail;
    while ( CAN1_Tx_Head == CAN1_Tx_Tail );//Ϊ��ֹ���ݻ��Ҷ�Ū�ϵ�Ӳ�ȴ�
    tmptail = ( CAN1_Tx_Tail + 1 ) & CAN1_TX_BUF_MARK;
    CAN1_Tx_Tail = tmptail;
    return &CAN1_TX_Buf[tmptail];
}

/****************************************************************************************************
��������: CAN1BeginSend/StopSend
��������: ��������
�������: ��
���ز���: ��
��   ע: ����ʹ�ÿ��жϷ�ʽ����,ֻҪ���ͼĴ���Ϊ��,�����뷢�Ϳ��ж�,ϵͳ�����ж��н��з���
****************************************************************************************************/
void CAN1BeginSend(void)
{
    CAN1->IER |= CAN_IT_TME;//����������ж�
}

void CAN1StopSend(void)
{
    CAN1->IER &= ~CAN_IT_TME;//����������ж�
}

/****************************************************************************************************
��������: CAN1PutDatatoRxBuf()
��������: �����ݷŽ�������
�������: ��
���ز���: ��
��   ע: Ӧ���ڽ����ж��е��ô˺���
****************************************************************************************************/
void CAN1PutDatatoRxBuf(CanRxMsg *pRxMessage)
{
    u32 tmphead;
    tmphead = ( CAN1_Rx_Head + 1 ) & CAN1_RX_BUF_MARK;//����ͷ�����ֵ�ж�,�������,����0
    CAN1_Rx_Head = tmphead; 	// ÿ��һ������,����ͷ����1
    CAN1_RX_Buf[tmphead] = *pRxMessage;
}

/****************************************************************************************************
��������: CAN1IsDataInRxBuf()
��������: ��֪�������Ƿ�������
�������: ��
���ز���: ��
��   ע: �����е�ͷ��β����ȵ�ʱ��,�ʹ������������ڻ�����
****************************************************************************************************/
bool CAN1IsDataInRxBuf( void )
{
    if( CAN1_Rx_Head == CAN1_Rx_Tail )
        return FALSE;
    else
        return TRUE;
}

/****************************************************************************************************
��������: CAN1GetRxBufLen()
��������: ��ȡ��������Ч���ݵĳ���
�������: ��
���ز���: ��
��   ע: ��ȡ��ֵ��ȻΪ��Сֵ,��Ϊ��ʵ���Ȼ᲻�ϱ仯.������32λ��ARMϵͳ,��ȡ32λ���ݲ������ٽ�����,���Կ��Բ����ǹ��ж�
****************************************************************************************************/
u32 CAN1GetRxBufLen(void)
{
    if(CAN1_Rx_Head >= CAN1_Rx_Tail)
    {
        return(CAN1_Rx_Head - CAN1_Rx_Tail);
    }
    else
    {
        return(CAN1_RX_BUF_MAX + CAN1_Rx_Head - CAN1_Rx_Tail);
    }
}

/****************************************************************************************************
��������: CAN1GetRxBufDat()
��������: �Ӷ����л�ȡ����
�������: ��
���ز���: ��
��   ע: ���ô˺���ǰ����ȷ��������������!!���������Ӳ�ȴ�
****************************************************************************************************/
CanRxMsg *CAN1GetRxBufDat( void )
{
    u32 tmptail;
    while ( CAN1_Rx_Head == CAN1_Rx_Tail );//Ϊ��ֹ���ݻ��Ҷ�Ū�ϵ�Ӳ�ȴ�
    tmptail = ( CAN1_Rx_Tail + 1 ) & CAN1_RX_BUF_MARK;//�����е�Ŀ������ʼ��ʹCAN1_Rx_Tail��0��CAN1_RX_BUF_MARK֮��ķ�Χѭ��
    CAN1_Rx_Tail = tmptail;                           //
    return &CAN1_RX_Buf[tmptail];
}


/****************************************************************************************************
��������: CanCmdManage()
��������: ����CAN������Ϣ
�������: CanRxMsg *pRxMessage, CanTxMsg *pTxMessage
���ز���: EN_BOOL����ȷ�򷵻�TRUE�����󷵻�FALSE
��   ע:  ������ˢ����CanЭ����о��򣬽������������ؽ���ͨѶ������ͨѶAPI
****************************************************************************************************/
//bool CanCmdManage(CanRxMsg *pRxMessage, CanTxMsg *pTxMessage)
//{
//	UN_CAN_ID_EXT unCanIdExtTmp;
//	unCanIdExtTmp.uiExtID = pRxMessage->ExtId;
//
//	switch(unCanIdExtTmp.stExtID.Channel)
//	{
//		case 0x03://������Ʋ�����ѯ
//		{
//			switch(unCanIdExtTmp.stExtID.Property)
//			{
//				case 0x00://��ȡ����λ����Ϣ
//					pTxMessage->DLC = 4;
//					*((SINT32*)(&pTxMessage->Data[0])) = stMotorCtrl.stCoder.siCurCodeNum;
//					break;
//				case 0x01://����ٶȲ�ѯ
//					pTxMessage->DLC = 4;
//					*(FP32*)&pTxMessage->Data[0] = stMotorCtrl.fpVeltFB;
//					break;
//				case 0x02://���������ѯ
//					pTxMessage->DLC = 2;
//					*(SSHORT16*)&pTxMessage->Data[0] = stMotorCtrl.ssCrtFB;
//					break;
//			}
//			break;
//		}
//
//		case 0x09://����������
//			switch(unCanIdExtTmp.stExtID.Property)
//			{
//				case 0x41://������ʧ��
//					stMotorCtrl.emDriverState = OFF;
//					pTxMessage->DLC = 0;
//					break;
//				case 0x42://������ʹ��
//					stMotorCtrl.emDriverState = ON;
//					pTxMessage->DLC = 0;
//					break;
//				case 0x43://����λ������
//					stMotorCtrl.stCoder.emRunState = FIRST;
//					stMotorCtrl.stCoder.siCodeCycleNum = 0;
//					stMotorCtrl.stCoder.siCurCodeNum = 0;
//					stMotorCtrl.stCoder.siDetCodeNum = 0;
//					stMotorCtrl.stCoder.siPreCodeNum = 0;
//					TIM4->CNT = 0;
//					pTxMessage->DLC = 1;//���óɹ�
//					pTxMessage->Data[0] = 0xaa;
//					break;
//			}
//			break;
//
//		case 0x11://����λ������
//			stMotorCtrl.stPosCtrl.siPosEnd = *(SINT32*)&pRxMessage->Data[0];
//			stMotorCtrl.stPosCtrl.siPosDes = stMotorCtrl.stCoder.siCurCodeNum;
//			stMotorCtrl.stPosCtrl.fpVeltDes = stMotorCtrl.fpVeltFB * stMotorCtrl.stMotorPara.usNe * stMotorCtrl.stMotorPara.fpI / 15;
//
//			if(pRxMessage->DLC == 8)
//			{
//				stMotorCtrl.stPosCtrl.siVeltMax = *(USHORT16*)&pRxMessage->Data[4];
//				stMotorCtrl.stPosCtrl.fpAcc = *(USHORT16*)&pRxMessage->Data[6];
//			}
//			stMotorCtrl.emCtrlMode = POS;
//
//			pTxMessage->DLC = 1;
//			pTxMessage->Data[0] = 0xaa;
//			break;
//		case 0x12://���λ������
//			stMotorCtrl.stPosCtrl.siPosEnd += *(SINT32*)&pRxMessage->Data[0];
//			stMotorCtrl.stPosCtrl.siPosDes = stMotorCtrl.stCoder.siCurCodeNum;
//			stMotorCtrl.stPosCtrl.fpVeltDes = stMotorCtrl.fpVeltFB * stMotorCtrl.stMotorPara.usNe * stMotorCtrl.stMotorPara.fpI / 15;
//
//		  stMotorCtrl.emCtrlMode = POS;
//
//			pTxMessage->DLC = 1;
//			pTxMessage->Data[0] = 0xaa;
//
//			break;
//
//		case 0x13://�����ٶ�����
//		{
//			FP32 fpTmp = *(FP32*)&pRxMessage->Data[0];
//			if(fpTmp < stMotorCtrl.stMotorPara.siMaxVelt && fpTmp > stMotorCtrl.stMotorPara.siMinVelt)
//			{
//				stMotorCtrl.emCtrlMode = VELT;
//				stMotorCtrl.fpVeltDes = fpTmp;
//				pTxMessage->Data[0] = 0xaa;
//			}
//			else
//			{
//				pTxMessage->Data[0] = 0xee;
//			}
//			pTxMessage->DLC = 1;
//			break;
//		}
//
//		case 0x14://��������ģʽ
//		{
//			SSHORT16 fpTmpCrt = *(FP32*)&pRxMessage->Data[0];

//			if(fabs(fpTmpCrt) < stMotorCtrl.stMotorPara.usMaxI)
//			{
//				stMotorCtrl.ssCrtDes = fpTmpCrt;
//				stMotorCtrl.emCtrlMode = CURRENT;
//				//���óɹ�
//				pTxMessage->Data[0] = 0xaa;
//			}
//			else
//			{
//				pTxMessage->Data[0] = 0x55;
//			}
//			pTxMessage->DLC = 1;
//			break;
//		}
//
//		case 0x19://��������ģʽ
//		{
//			SSHORT16 ssTmpPWMDuty = *(SSHORT16*)&pRxMessage->Data[0];
//			if(fabs(ssTmpPWMDuty) < stMotorCtrl.usMaxPwmDuty)
//			{
//				stMotorCtrl.ssPwmDuty = ssTmpPWMDuty;
//				stMotorCtrl.emCtrlMode = OPEN_LOOP;
//
//				//���óɹ�
//				pTxMessage->Data[0] = 0xaa;
//			}
//			else
//			{
//				pTxMessage->Data[0] = 0x55;
//			}
//			pTxMessage->DLC = 1;
//			break;
//		}

//		case 0x1B://λ�ÿ��Ʋ�������
//		{
//			switch(unCanIdExtTmp.stExtID.Property)
//			{
//				case 0x02://����ٶ�
//					stMotorCtrl.stPosCtrl.siVeltMax = *(SINT32*)&pRxMessage->Data[0];
//					break;
//				case 0x03://���ٶ�
//					stMotorCtrl.stPosCtrl.fpAcc = *(SINT32*)&pRxMessage->Data[0];
//					break;
//			}
//			pTxMessage->DLC = 1;//���óɹ�
//			pTxMessage->Data[0] = 0xaa;
//		}
//		break;

//		case 0x30://���Ϳ���
//			stMotorCtrl.unCanSendFlag.stSendFlag.SendPosFlag = pRxMessage->Data[0];
//			stMotorCtrl.unCanSendFlag.stSendFlag.SendVeltFlag = pRxMessage->Data[1];
//			stMotorCtrl.unCanSendFlag.stSendFlag.SendCrtFlag = pRxMessage->Data[2];
//			break;

//		default:
//			return FALSE;
//	}
//	unCanIdExtTmp.stExtID.DeviceID = stMotorCtrl.uiSlaveAddr;
//	pTxMessage->ExtId = unCanIdExtTmp.uiExtID;
//	pTxMessage->IDE = CAN_ID_EXT;
//	pTxMessage->RTR = CAN_RTR_DATA;
//	return TRUE;
//}
