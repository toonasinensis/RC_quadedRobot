#include "sys.h"
#include "delay.h"
#include "usart.h" 
#include "led.h"
#include "key.h"
#include "lcd.h"
#include "sdram.h"
//#include "usmart.h"
#include "pcf8574.h"
#include "timer.h"
#include "mpu.h"
#include "malloc.h"
#include "lwip/netif.h"
#include "lwip_comm.h"
#include "lwipopts.h"
#include "udp_demo.h"
#include "gpio.h"
#include "dma.h"
#include "unitree_motor_ctrl_task.h"

struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{ 	
	while((UART7->ISR&0X40)==0);//ѭ������,ֱ���������   
	UART7->TDR=(u8)ch;      
	return ch;
}

extern u8 udp_demo_flag;  //UDP ����ȫ��״̬��Ǳ���


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
     //   usart1_start_recv();
    }
    if (huart->Instance == UART5)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
       // usart5_start_recv();
    }
}






int main(void)
{
    u8 key=0;
	
    Write_Through();                        //����ǿ��͸д��
    MPU_Memory_Protection();                //������ش洢����
    Cache_Enable();                         //��L1-Cache
	HAL_Init();				        		            //��ʼ��HAL��
	Stm32_Clock_Init(160,5,2,4);  		        //����ʱ��,400Mhz 
	delay_init(400);						              //��ʱ��ʼ��
	
	MX_GPIO_Init();
  MX_DMA_Init();
	MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_UART4_Init();
  MX_UART5_Init();
	MX_USART6_UART_Init();
  MX_UART7_Init();

	
	__HAL_USART_ENABLE_IT(&huart1,UART_IT_IDLE);
	__HAL_USART_ENABLE_IT(&huart2,UART_IT_IDLE);
	__HAL_USART_ENABLE_IT(&huart3,UART_IT_IDLE);
	__HAL_USART_ENABLE_IT(&huart4,UART_IT_IDLE);
	__HAL_USART_ENABLE_IT(&huart5,UART_IT_IDLE);
	__HAL_USART_ENABLE_IT(&huart6,UART_IT_IDLE);
	__HAL_USART_ENABLE_IT(&huart7,UART_IT_IDLE);
	
//  __HAL_USART_ENABLE_IT(&huart1,UART_IT_RXNE);
	
HAL_UART_Receive_DMA(&huart1, uart_rx_buffer[0], UART_RX_LEN);
HAL_UART_Receive_DMA(&huart2, uart_rx_buffer[1], UART_RX_LEN);
HAL_UART_Receive_DMA(&huart3, uart_rx_buffer[2], UART_RX_LEN);
HAL_UART_Receive_DMA(&huart4, uart_rx_buffer[3], UART_RX_LEN);
HAL_UART_Receive_DMA(&huart5, uart_rx_buffer[4], UART_RX_LEN);
HAL_UART_Receive_DMA(&huart6, uart_rx_buffer[5], UART_RX_LEN);
HAL_UART_Receive_DMA(&huart7, uart_rx_buffer[6], UART_RX_LEN);


	
	LED_Init();								//��ʼ��LED
	KEY_Init();								//��ʼ������
	SDRAM_Init();             //��ʼ��SDRAM
	//LCD_Init();								//��ʼ��LCD
	PCF8574_Init();                         //��ʼ��PCF8574
  my_mem_init(SRAMIN);		            //��ʼ���ڲ��ڴ��
	my_mem_init(SRAMEX);		            //��ʼ���ⲿ�ڴ��
	my_mem_init(SRAMDTCM);		          //��ʼ��DTCM�ڴ��
   
	
  TIM3_Init(100-1,2000-1);               //��ʱ��3��ʼ������ʱ��ʱ��Ϊ200M����Ƶϵ��Ϊ2000-1��
                                          //���Զ�ʱ��3��Ƶ��Ϊ200M/2000=100K���Զ���װ��Ϊ100-1����ô��ʱ�����ھ���1ms
	while(lwip_comm_init())                 //lwip��ʼ��
	{
	//if failed....
	}
//	
//	modify(&motor1, send_buff_uart);

		udp_demo_test();  		//UDP ģʽ

	while(1)
	{

				
	}

//	
//	
//	while(1)
//	{
//        lwip_periodic_handle();	//LWIP�ں���Ҫ��ʱ����ĺ���
//        key=KEY_Scan(0);
//		if(key==KEY1_PRES)		    //��KEY1����������
//		{
//			if((udp_demo_flag & 1<<5)){}// printf("UDP�����Ѿ�����,�����ظ�����\r\n");	//������ӳɹ�,�����κδ���
//			else udp_demo_test();		//���Ͽ����Ӻ�,����udp_demo_test()����
//		}
//		delay_ms(10);
//	}  
}

