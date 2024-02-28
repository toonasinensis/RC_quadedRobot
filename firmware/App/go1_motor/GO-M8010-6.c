#include "usart.h"
#include "delay.h"
#include "motor_control.h"
#include "crc_ccitt.h"
#include "stdio.h"

int modify_data(MOTOR_send *motor_s)
{
    motor_s->hex_len = 17;
    motor_s->motor_send_data.head[0] = 0xFE;
    motor_s->motor_send_data.head[1] = 0xEE;
	
	SATURATE(motor_s->id,   0,    15);
	SATURATE(motor_s->mode, 0,    7);
	SATURATE(motor_s->K_P,  0.0f,   25.599f);
	SATURATE(motor_s->K_W,  0.0f,   25.599f);
	SATURATE(motor_s->T,   -127.99f,  127.99f);
	SATURATE(motor_s->W,   -804.00f/6.33f,  804.00f/6.33f);
	SATURATE(motor_s->Pos, -411774.0f/6.33f,  411774.0f/6.33f);

    motor_s->motor_send_data.mode.id      = motor_s->id;
    motor_s->motor_send_data.mode.status  = motor_s->mode;
    motor_s->motor_send_data.comd.k_pos   = motor_s->K_P/25.6f*32768;     
    motor_s->motor_send_data.comd.k_spd   = motor_s->K_W/25.6f*32768;
    motor_s->motor_send_data.comd.pos_des = motor_s->Pos/6.2832f*32768*6.33f;       //motor_s->Pos单位rad
    motor_s->motor_send_data.comd.spd_des = motor_s->W/6.2832f*256*6.33f;           //motor_s->W单位rad/s
    motor_s->motor_send_data.comd.tor_des = motor_s->T*256;                   //motor_s->T单位N.m
    motor_s->motor_send_data.CRC16 = crc_ccitt(0, (uint8_t *)&motor_s->motor_send_data, 15);
    return 0;
}

int extract_data(MOTOR_recv *motor_r,uint8_t *raw_data)
{
	memcpy(&motor_r->motor_recv_data,raw_data,16);
    if(motor_r->motor_recv_data.CRC16 != crc_ccitt(0, (uint8_t *)&motor_r->motor_recv_data, 14))
    {
        motor_r->correct = 0;
        return motor_r->correct;
    }
    else
	{
        motor_r->motor_id  = motor_r->motor_recv_data.mode.id;
        motor_r->mode      = motor_r->motor_recv_data.mode.status;
        motor_r->Temp      = motor_r->motor_recv_data.fbk.temp;
        motor_r->MError    = motor_r->motor_recv_data.fbk.MError;
        motor_r->W         = ((float)motor_r->motor_recv_data.fbk.speed/256)*6.2832f/6.33f;      //motor_s->W单位rad/s
        motor_r->T         = ((float)motor_r->motor_recv_data.fbk.torque)/256;                  //motor_s->T单位N.m
        motor_r->Pos       = 6.2832f*((float)motor_r->motor_recv_data.fbk.pos)/32768/6.33f;      //motor_s->Pos单位rad
		motor_r->footForce = motor_r->motor_recv_data.fbk.force;
		motor_r->correct   = 1;
        return motor_r->correct;
    }
}
int uart_delay_time = 8;
int test_size_Left;
int test_size;


//void SendGO1_Motor_Data3(MOTOR_send *pData)
//{
//    modify_data(pData);
//    GPIO_SetBits(GPIOB, GPIO_Pin_0);
//	test_size_Left = sizeof(pData->motor_send_data);
//	delay_us(uart_delay_time);
//    USART3_DMA_Tx(&(pData->motor_send_data),17); 
//	while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET); //等待发送结束	
//    GPIO_ResetBits(GPIOB, GPIO_Pin_0);
//	delay_us(uart_delay_time);
//}


void SendGO1_Motor_Data(MOTOR_send *  pData,uint8_t *raw_data)
{
    modify_data(pData);	 
	memcpy(raw_data,&pData->motor_send_data,17);
}

