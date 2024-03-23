#ifndef __VERIFY_H__
#define __VERIFY_H__

#include "stm32f4xx.h"


u32 Verify_CRC8_Check_Sum(u8 *pchMessage, u32 dwLength);
void Append_CRC8_Check_Sum(u8 *pchMessage, u32 dwLength);
u32 Verify_CRC16_Check_Sum(u8 *pchMessage, u32 dwLength);
void Append_CRC16_Check_Sum(u8 * pchMessage,u32 dwLength);

#endif
