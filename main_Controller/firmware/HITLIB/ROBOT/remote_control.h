#ifndef __REMOTE_CONTROL_H__
#define __REMOTE_CONTROL_H__

#include <string.h>
#include "uart_protocol.h"
//#include "system_monitor_task.h"
#include "navigation_algorithm.h"
#include "filter_algorithm.h"
#include "math.h"

//    ����ֵ  20 19 23 22 21
//           0x14 0x0E 0x08    0x01 0x1F 0x19
//           0x18 0x12 0x0C    0x05 0x23 0x1D
//           0x17 0x11 0x0B    0x04 0x22 0x1C
// 0x16 0x0F 0x10 0x09 0x0A    0x03 0x02 0x21 0x20 0x1B
// 0x15 0x13 0x0D 0x07 0x25    0x26 0x06 0x24 0x1E 0x1A
//


//���ֱ�Ϊ���ģʽʱWlan_JOYSTICK_STATEֵΪ0x73������Ϊ0x41



//qrpucp:ȥ���˱���ǰ��volatile
//ff:��ҡ������tem_buf[5],��255 ��0
//   ��ҡ������tem_buf[6],��255 ��0
//ff:��ҡ������tem_buf[7],��255 ��0
//   ��ҡ������tem_buf[8],��255 ��0

#define Wlan_JOYSTICK_RESERVED 		(((uint16_t)tmp_buf[3])<<8|tmp_buf[4])
#define Wlan_JOYSTICK_LEFT 			(((uint16_t)tmp_buf[5])<<8|tmp_buf[6])
#define Wlan_JOYSTICK_RIGTH 		(((uint16_t)tmp_buf[7])<<8|tmp_buf[8])
#define Wlan_JOYSTICK_STATE 		tmp_buf[1]

#define Wlan_PSKEY 	            (((uint16_t)(0xFF<<8)|tmp_buf_11))
#define	 JS_MID_POS 132//ҡ���м�ֵ     

/*�ֱ���ؽṹ��*/
typedef struct
{
    uint16_t  	usJsKey;
	uint8_t   	aucKeyPress[16];           //Ϊ1ʱ����ʾ����������Ч
	uint32_t    auiPressDuration[16];      //���³���ʱ�����ֵ�Ƚϣ��ж��Ƿ�Ϊ����
	uint32_t    uiStartTime[16];           
	uint32_t    uiCurTime[16];
	uint16_t  	usJsState;
	uint16_t 	usJsLeft;
	uint16_t  	usJsRight;	
}ST_JS_VALUE;

extern void ReadWlanJsValue(ST_JS_VALUE *pstJsValue);
extern void ReadWlanKeyValue(uint16_t *Wlan_pusKeyValue);
extern void  CalSpeedByJoyStick(ST_JS_VALUE *g_stJsValue, cNav *p_nav, uint8_t ucGateX,
								int16_t ssXSpedLimit,uint8_t ucGateY,int16_t ssYSpedLimit,
													uint8_t ucGateW,int16_t ssWSpedLimit);
extern ST_JS_VALUE 		g_stJsValue;
extern ST_JS_VALUE 		Wlan_g_stJsValue;

extern int flag_coordinate;
#endif
