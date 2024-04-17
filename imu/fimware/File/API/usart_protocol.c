#include "usart_protocol.h"
#include "gimbal_task.h"
#include "bmi088_driver.h"

u8 UA1RxDMAbuf[UA1RxDMAbuf_LEN] = {0};
u8 UA2RxDMAbuf[UA2RxDMAbuf_LEN] = {0};
u8 UA3RxDMAbuf[UA3RxDMAbuf_LEN] = {0};
u8 UA4RxDMAbuf[UA4RxDMAbuf_LEN] = {0};
u8 UA6RxDMAbuf[UA6RxDMAbuf_LEN] = {0};
/*--����4�����أ�����Э�����ʹ�õı���*/
ST_IMU G_ST_IMU = {0};

//�ǶȽ����-180~+180
float Angle_Inf_To_180(float angle)
{
    if(fabs(angle)>1800000) return 0;
    else
    {
        int temp = ((int)angle)/360;
        angle -= 360.0f*temp;
        if(angle>+180) angle-=360;
        else if(angle<-180) angle+=360;
        return angle;
    }
}

extern FP32 Acc_X_Ori;
extern FP32 Acc_Y_Ori;
extern FP32 Acc_Z_Ori;
int Catch_Cnt ;
/*----------------------------Э���������----------------------------------------------------*/
//����4���ʹ���������>>>����
inline void MainControl_Tx_Protocol(void)
{
    G_ST_IMU.head[0]           = 0x55;
		G_ST_IMU.head[1]						= 0x00 ;
	
//		G_ST_IMU.Send.pitch_angle       = PitchPosFB_Gyro;
//    G_ST_IMU.Send.pitch_speed       = PitchSpeedFB;
//	
//    G_ST_IMU.Send.yaw_angle         = YawPosFB_Gyro;
//    G_ST_IMU.Send.yaw_speed         = YawSpeedFB;
//    G_ST_IMU.Send.distence_X        = X_Axis_Distance;
//    G_ST_IMU.Send.distence_Y        = Y_Axis_Distance;
//    G_ST_IMU.Send.distence_Z        = Z_Axis_Distance;
			G_ST_IMU.ang_vel_x  = Gyro_X_Ori;
			G_ST_IMU.ang_vel_y  = Gyro_Y_Ori;
			G_ST_IMU.ang_vel_z  = Gyro_Z_Ori;
			
			G_ST_IMU.quat_W   =  q0_send;
			G_ST_IMU.quat_X   = q1_send;
			G_ST_IMU.quat_Y   = q2_send;
			G_ST_IMU.quat_Z   = q3_send;
			
			G_ST_IMU.acc_x_send   = Acc_X_Ori/1000;
			G_ST_IMU.acc_y_send   = Acc_Y_Ori/1000;
			G_ST_IMU.acc_z_send  = Acc_Z_Ori/1000;
	
   //Append_CRC16_Check_Sum(G_ST_IMU.head, UA4TxDMAbuf_LEN);

    //while(DMA_GetCurrDataCounter(DMA1_Stream4));        //��֮ǰ�ķ���
    DMA_ClearITPendingBit(DMA1_Stream4, DMA_IT_TCIF4);  //����DMA_Mode_Normal,����û��ʹ������ж�ҲҪ������������ֻ��һ��

    DMA_Cmd(DMA1_Stream4, DISABLE);				        //���õ�ǰ����ֵǰ�Ƚ���DMA
    DMA1_Stream4->M0AR = (u32)&G_ST_IMU;     //���õ�ǰ�������ݻ���ַ:Memory0 tARget
    DMA1_Stream4->NDTR = (u32)UA4TxDMAbuf_LEN;    //���õ�ǰ���������ݵ�����:Number of Data units to be TRansferred
    DMA_Cmd(DMA1_Stream4, ENABLE);				        //���ô���DMA����
}
//����4���մ�������>>>������
bool Start_Catch_Flag = FALSE ;
inline void MainControl_Rx_Protocol(void)
{
//    if( UA4RxDMAbuf[0] == 0x55 &&                                  //֡ͷУ��
//        UA4RxDMAbuf[1] == 0x00 &&                                  //֡ͷУ��
//				Verify_CRC16_Check_Sum(UA4RxDMAbuf, UA4RxDMAbuf_LEN) )  //֡β����
//    {
//				Start_Catch_Flag = TRUE ;
//        memcpy(&G_ST_IMU.Receive.ReloadStatus,      &UA4RxDMAbuf[2], 1); //1
//        memcpy(&G_ST_IMU.Receive.Restart,           &UA4RxDMAbuf[3], 1); //1

//        if(G_ST_IMU.Receive.Restart==0xF)//���յ����ֱ�Ӹ�λ
//        {
//            NVIC_SystemReset();
//        }
//    }
}

/********************************************************����2ͨѶЭ��***********************************************/
usart2_tx_protocol_t usart2_eft =
 {
 	0x55,
 	0x11,
	(uint8_t)52359358,//����id 
 	USART2_TX_DATA_LEN, //������50
 	{0},
 	0x22,
 	0xAA
 };

usart2_rx_protocol_t usart2_efr =
{
	0x55,
	0x00,
	(uint8_t)52359358,//����id
	USART2_RX_DATA_LEN,
	{0},
	0x00,
	0xAA
};
//�ƿء������
void Camera_Tx_Protocol(void)
{
	//������Ԫ��
	  memcpy(&usart2_eft.num[0],&q0_send,4);
	  memcpy(&usart2_eft.num[4],&q1_send,4);
		memcpy(&usart2_eft.num[8],&q2_send,4);
	  memcpy(&usart2_eft.num[12],&q3_send,4);
	//���ͽ��ٶ�
	  memcpy(&usart2_eft.num[16],&Gyro_X_Real,4);
		memcpy(&usart2_eft.num[20],&Gyro_Y_Real,4);
		memcpy(&usart2_eft.num[24],&Gyro_Z_Real,4);
	//����������ٶ�
	  memcpy(&usart2_eft.num[28],&Acc_X_Send,4);
		memcpy(&usart2_eft.num[32],&Acc_Y_Send,4);
  	memcpy(&usart2_eft.num[36],&Acc_Z_Send,4);
    DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_TCIF6);  //����DMA_Mode_Normal,����û��ʹ������ж�ҲҪ������������ֻ��һ��
    DMA_Cmd(DMA1_Stream6, DISABLE);				        //���õ�ǰ����ֵǰ�Ƚ���DMA
    DMA1_Stream6->M0AR = (u32)&usart2_eft;        //���õ�ǰ�������ݻ���ַ:Memory0 tARget
    DMA1_Stream6->NDTR = (u32)sizeof(usart2_eft);    //���õ�ǰ���������ݵ�����:Number of Data units to be TRansferred
    DMA_Cmd(DMA1_Stream6, ENABLE);				        //���ô���DMA����   
}

