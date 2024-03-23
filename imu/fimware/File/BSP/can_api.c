#include "main.h"
#include "math.h"
#include "can_api.h"
#include "flash_api.h"
#include "pmsm_control.h"
#include "pmsm_control_types.h"


static CanRxMsg CAN1_RX_Buf[CAN1_RX_BUF_MAX];		//队列
static CanTxMsg CAN1_TX_Buf[CAN1_TX_BUF_MAX];		//队列

volatile u32 CAN1_Rx_Head = 0;					//队列头-接收的时候移动
volatile u32 CAN1_Rx_Tail = 0;					//队列尾-读取的时候移动

volatile u32 CAN1_Tx_Head = 0;					//队列头-接收的时候移动
volatile u32 CAN1_Tx_Tail = 0;					//队列尾-读取的时候移动

/****************************************************************************************************
函数名称: CAN1PutDatatoTxBuf()
函数功能: 把数据放进发送队列中
输入参数: CanTxMsg *pTxMessage
返回参数: 无
备   注: 用户需要有数据发的时候使用
****************************************************************************************************/
void CAN1PutDatatoTxBuf(CanTxMsg *pTxMessage)
{
    u32 tmphead;
    tmphead = ( CAN1_Tx_Head + 1 ) & CAN1_TX_BUF_MARK;//队列末端判断,到达末端,则变回0
    CAN1_Tx_Head = tmphead; 	// 每入列,队列头增加1
    CAN1_TX_Buf[tmphead] = *pTxMessage;

}

/****************************************************************************************************
函数名称: CAN1IsDataInTxBuf()
函数功能: 获知缓冲中是否有数据
输入参数: 无
返回参数: 无
备   注: 当队列的头和尾不相等的时候,就代表了有数据在缓冲中
****************************************************************************************************/
bool CAN1IsDataInTxBuf( void )
{
    if( CAN1_Tx_Head == CAN1_Tx_Tail )
        return FALSE;
    else
        return TRUE;
}

/****************************************************************************************************
函数名称: CAN1GetTxBufLen()
函数功能: 获取缓冲中有效数据的长度
输入参数: 无
返回参数: 无
备   注: :获取的值必然为最小值,因为真实长度会不断变化.由于是32位的ARM系统,获取32位数据不存在临界问题,所以可以不考虑关中断
**			所谓有效,是指剩余的可用长度
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
函数名称: CAN1GetTxBufDat()
函数功能: 从队列中获取数据
输入参数: 无
返回参数: 无
备   注:  调用此函数前请先确保队列中有数据!!
****************************************************************************************************/
CanTxMsg *CAN1GetTxBufDat( void )
{
    u32 tmptail;
    while ( CAN1_Tx_Head == CAN1_Tx_Tail );//为防止数据混乱而弄上的硬等待
    tmptail = ( CAN1_Tx_Tail + 1 ) & CAN1_TX_BUF_MARK;
    CAN1_Tx_Tail = tmptail;
    return &CAN1_TX_Buf[tmptail];
}

/****************************************************************************************************
函数名称: CAN1BeginSend/StopSend
函数功能: 启动发送
输入参数: 无
返回参数: 无
备   注: 这里使用空中断方式发送,只要发送寄存器为空,则会进入发送空中断,系统再在中断中进行发送
****************************************************************************************************/
void CAN1BeginSend(void)
{
    CAN1->IER |= CAN_IT_TME;//发送邮箱空中断
}

void CAN1StopSend(void)
{
    CAN1->IER &= ~CAN_IT_TME;//发送邮箱空中断
}

/****************************************************************************************************
函数名称: CAN1PutDatatoRxBuf()
函数功能: 把数据放进队列中
输入参数: 无
返回参数: 无
备   注: 应该在接收中断中调用此函数
****************************************************************************************************/
void CAN1PutDatatoRxBuf(CanRxMsg *pRxMessage)
{
    u32 tmphead;
    tmphead = ( CAN1_Rx_Head + 1 ) & CAN1_RX_BUF_MARK;//队列头的最大值判断,到达最大,则变回0
    CAN1_Rx_Head = tmphead; 	// 每收一次数据,队列头增加1
    CAN1_RX_Buf[tmphead] = *pRxMessage;
}

/****************************************************************************************************
函数名称: CAN1IsDataInRxBuf()
函数功能: 获知缓冲中是否有数据
输入参数: 无
返回参数: 无
备   注: 当队列的头和尾不相等的时候,就代表了有数据在缓冲中
****************************************************************************************************/
bool CAN1IsDataInRxBuf( void )
{
    if( CAN1_Rx_Head == CAN1_Rx_Tail )
        return FALSE;
    else
        return TRUE;
}

/****************************************************************************************************
函数名称: CAN1GetRxBufLen()
函数功能: 获取缓冲中有效数据的长度
输入参数: 无
返回参数: 无
备   注: 获取的值必然为最小值,因为真实长度会不断变化.由于是32位的ARM系统,获取32位数据不存在临界问题,所以可以不考虑关中断
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
函数名称: CAN1GetRxBufDat()
函数功能: 从队列中获取数据
输入参数: 无
返回参数: 无
备   注: 调用此函数前请先确保队列中有数据!!否则会陷入硬等待
****************************************************************************************************/
CanRxMsg *CAN1GetRxBufDat( void )
{
    u32 tmptail;
    while ( CAN1_Rx_Head == CAN1_Rx_Tail );//为防止数据混乱而弄上的硬等待
    tmptail = ( CAN1_Rx_Tail + 1 ) & CAN1_RX_BUF_MARK;//这两行的目的在于始终使CAN1_Rx_Tail在0到CAN1_RX_BUF_MARK之间的范围循环
    CAN1_Rx_Tail = tmptail;                           //
    return &CAN1_RX_Buf[tmptail];
}


/****************************************************************************************************
函数名称: CanCmdManage()
函数功能: 处理CAN总线消息
输入参数: CanRxMsg *pRxMessage, CanTxMsg *pTxMessage
返回参数: EN_BOOL，正确则返回TRUE，错误返回FALSE
备   注:  根据有刷驱动Can协议进行精简，仅仅用于与主控进行通讯，主控通讯API
****************************************************************************************************/
//bool CanCmdManage(CanRxMsg *pRxMessage, CanTxMsg *pTxMessage)
//{
//	UN_CAN_ID_EXT unCanIdExtTmp;
//	unCanIdExtTmp.uiExtID = pRxMessage->ExtId;
//
//	switch(unCanIdExtTmp.stExtID.Channel)
//	{
//		case 0x03://电机控制参数查询
//		{
//			switch(unCanIdExtTmp.stExtID.Property)
//			{
//				case 0x00://获取绝对位置信息
//					pTxMessage->DLC = 4;
//					*((SINT32*)(&pTxMessage->Data[0])) = stMotorCtrl.stCoder.siCurCodeNum;
//					break;
//				case 0x01://电机速度查询
//					pTxMessage->DLC = 4;
//					*(FP32*)&pTxMessage->Data[0] = stMotorCtrl.fpVeltFB;
//					break;
//				case 0x02://电机电流查询
//					pTxMessage->DLC = 2;
//					*(SSHORT16*)&pTxMessage->Data[0] = stMotorCtrl.ssCrtFB;
//					break;
//			}
//			break;
//		}
//
//		case 0x09://控制器命令
//			switch(unCanIdExtTmp.stExtID.Property)
//			{
//				case 0x41://驱动器失能
//					stMotorCtrl.emDriverState = OFF;
//					pTxMessage->DLC = 0;
//					break;
//				case 0x42://驱动器使能
//					stMotorCtrl.emDriverState = ON;
//					pTxMessage->DLC = 0;
//					break;
//				case 0x43://绝对位置清零
//					stMotorCtrl.stCoder.emRunState = FIRST;
//					stMotorCtrl.stCoder.siCodeCycleNum = 0;
//					stMotorCtrl.stCoder.siCurCodeNum = 0;
//					stMotorCtrl.stCoder.siDetCodeNum = 0;
//					stMotorCtrl.stCoder.siPreCodeNum = 0;
//					TIM4->CNT = 0;
//					pTxMessage->DLC = 1;//设置成功
//					pTxMessage->Data[0] = 0xaa;
//					break;
//			}
//			break;
//
//		case 0x11://绝对位置运行
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
//		case 0x12://相对位置运行
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
//		case 0x13://绝对速度运行
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
//		case 0x14://电流控制模式
//		{
//			SSHORT16 fpTmpCrt = *(FP32*)&pRxMessage->Data[0];

//			if(fabs(fpTmpCrt) < stMotorCtrl.stMotorPara.usMaxI)
//			{
//				stMotorCtrl.ssCrtDes = fpTmpCrt;
//				stMotorCtrl.emCtrlMode = CURRENT;
//				//设置成功
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
//		case 0x19://开环控制模式
//		{
//			SSHORT16 ssTmpPWMDuty = *(SSHORT16*)&pRxMessage->Data[0];
//			if(fabs(ssTmpPWMDuty) < stMotorCtrl.usMaxPwmDuty)
//			{
//				stMotorCtrl.ssPwmDuty = ssTmpPWMDuty;
//				stMotorCtrl.emCtrlMode = OPEN_LOOP;
//
//				//设置成功
//				pTxMessage->Data[0] = 0xaa;
//			}
//			else
//			{
//				pTxMessage->Data[0] = 0x55;
//			}
//			pTxMessage->DLC = 1;
//			break;
//		}

//		case 0x1B://位置控制参数设置
//		{
//			switch(unCanIdExtTmp.stExtID.Property)
//			{
//				case 0x02://最大速度
//					stMotorCtrl.stPosCtrl.siVeltMax = *(SINT32*)&pRxMessage->Data[0];
//					break;
//				case 0x03://加速度
//					stMotorCtrl.stPosCtrl.fpAcc = *(SINT32*)&pRxMessage->Data[0];
//					break;
//			}
//			pTxMessage->DLC = 1;//设置成功
//			pTxMessage->Data[0] = 0xaa;
//		}
//		break;

//		case 0x30://发送控制
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
