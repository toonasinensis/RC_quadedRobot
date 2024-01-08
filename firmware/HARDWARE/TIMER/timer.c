#include "timer.h"
#include "led.h"
//////////////////////////////////////////////////////////////////////////////////	 
#include "udp_comm.h"
#include "unitree_motor_ctrl_task.h"
#include "system_monitor.h"
TIM_HandleTypeDef TIM3_Handler;      //��ʱ����� 

extern u32 lwip_localtime;	         //lwip����ʱ�������,��λ:ms
//ͨ�ö�ʱ��3�жϳ�ʼ��,��ʱ��3��APB1�ϣ�APB1�Ķ�ʱ��ʱ��Ϊ200MHz
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//��ʱ�����ʱ����㷽��:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=��ʱ������Ƶ��,��λ:Mhz
//����ʹ�õ��Ƕ�ʱ��3!(��ʱ��3����APB1�ϣ�ʱ��ΪHCLK/2)
void TIM3_Init(u16 arr,u16 psc)
{  
    TIM3_Handler.Instance=TIM3;                          //ͨ�ö�ʱ��3
    TIM3_Handler.Init.Prescaler=psc;                     //��Ƶ
    TIM3_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //���ϼ�����
    TIM3_Handler.Init.Period=arr;                        //�Զ�װ��ֵ
    TIM3_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//ʱ�ӷ�Ƶ����
    HAL_TIM_Base_Init(&TIM3_Handler);
    
    HAL_TIM_Base_Start_IT(&TIM3_Handler); //ʹ�ܶ�ʱ��3�Ͷ�ʱ��3�����жϣ�TIM_IT_UPDATE    
}

//��ʱ���ײ�����������ʱ�ӣ������ж����ȼ�
//�˺����ᱻHAL_TIM_Base_Init()��������
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if(htim->Instance==TIM3)
	{
		__HAL_RCC_TIM3_CLK_ENABLE();            //ʹ��TIM3ʱ��
		HAL_NVIC_SetPriority(TIM3_IRQn,1,3);    //�����ж����ȼ�����ռ���ȼ�1�������ȼ�3
		HAL_NVIC_EnableIRQ(TIM3_IRQn);          //����ITM3�ж�   
	}  
}

//��ʱ��3�жϷ�����
void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&TIM3_Handler);
}
uint8_t start_send;
uint8_t if_long_use;
uint8_t fast_send=0;
//��ʱ��3�жϷ���������
uint32_t timer_pp;

uint32_t psc_1000;
extern float normal_frecquency;
extern u8 udp_send_flag;
float otto_pos;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim==(&TIM3_Handler))
    {
			
			psc_1000++;
			if(start_send||if_long_use||fast_send)
				{
					if(if_long_use&&timer_pp>1000)
					{
						timer_pp = 0;
						
						
					start_send = 0;
						
							send_all_motor_command(uart_tx_buffer,uart_rx_buffer,leg);
					}
					else if(!if_long_use)
					{
						
					send_all_motor_command(uart_tx_buffer,uart_rx_buffer,leg);

					}
					timer_pp++;
					otto_pos = leg[0].knee_motor.feedback.W;
				}
				
				if(psc_1000>1000)//��Ƶ��1s
				{
					normal_frecquency = 0;
		//			cal_fps_sys(&system_monitor);
					psc_1000 = 0;
						for(int i = 0;i<LEG_NUM;i++)
					{
						

								leg[i].hip_motor.real_rate =leg[i].hip_motor.temp_rate;
								leg[i].thigh_motor.real_rate =leg[i].thigh_motor.temp_rate;
								leg[i].knee_motor.real_rate =leg[i].knee_motor.temp_rate;
								leg[i].hip_motor.temp_rate = 0;
						leg[i].thigh_motor.temp_rate = 0;
						leg[i].knee_motor.temp_rate = 0; 
					}
				}
				
				//����udp�������ݣ�
				
				// update motor feedback data
        for (int i = 0; i < LEG_NUM; ++i)
        {
            raw_motor_type2udp_motor_type(&udp_send_data.udp_motor_receive[i * 3], &leg[i].hip_motor.feedback);
            raw_motor_type2udp_motor_type(&udp_send_data.udp_motor_receive[i * 3 + 1], &leg[i].thigh_motor.feedback);
            raw_motor_type2udp_motor_type(&udp_send_data.udp_motor_receive[i * 3 + 2], &leg[i].knee_motor.feedback);
        }
        // add crc
        udp_send_data.check_digit = crc32_core((uint8_t *)&udp_send_data, sizeof(udp_send_data) / 4 - 1);

				
				
				
			//	udp_send_flag = 1;

        lwip_localtime +=1; //��10
    }
}
