#ifndef __DJI_PROTOCOL_H__
#define __DJI_PROTOCOL_H__

#include "main.h"


extern s16 PitchCurrent;
extern s16 YawCurrent;
extern s16 ShooterCurrent;
extern s16 FrictionWheel_Left_Current;
extern s16 FrictionWheel_Right_Current;

void Can_Send_Current(void);
u8 Send_Current_To_Gimbal(CAN_TypeDef *CANx, s16 gmyaw_i, s16 gmpitch_i, s16 supply_i);
u8 Send_Current_To_FrictionWheel(CAN_TypeDef *CANx,s16 fw_right_i,s16 fw_left_i);
void Abs_Encoder_Process(volatile ST_ENCODER* encoder, s32 value);
s32 Get_Speed(CanRxMsg* rx_message);
s32 Get_Encoder_Number(CanRxMsg* rx_message);
float Get_DJI_GYRO(CanRxMsg* rxmessage);
float Get_Temperature(CanRxMsg* rx_message);

#endif
