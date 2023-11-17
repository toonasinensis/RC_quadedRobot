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
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((UART7->ISR&0X40)==0);//循环发送,直到发送完毕   
	UART7->TDR=(u8)ch;      
	return ch;
}

extern u8 udp_demo_flag;  //UDP 测试全局状态标记变量


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
	
    Write_Through();                        //开启强制透写！
    MPU_Memory_Protection();                //保护相关存储区域
    Cache_Enable();                         //打开L1-Cache
	HAL_Init();				        		            //初始化HAL库
	Stm32_Clock_Init(160,5,2,4);  		        //设置时钟,400Mhz 
	delay_init(400);						              //延时初始化
	
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


	
	LED_Init();								//初始化LED
	KEY_Init();								//初始化按键
	SDRAM_Init();             //初始化SDRAM
	//LCD_Init();								//初始化LCD
	PCF8574_Init();                         //初始化PCF8574
  my_mem_init(SRAMIN);		            //初始化内部内存池
	my_mem_init(SRAMEX);		            //初始化外部内存池
	my_mem_init(SRAMDTCM);		          //初始化DTCM内存池
   
	
  TIM3_Init(100-1,2000-1);               //定时器3初始化，定时器时钟为200M，分频系数为2000-1，
                                          //所以定时器3的频率为200M/2000=100K，自动重装载为100-1，那么定时器周期就是1ms
	while(lwip_comm_init())                 //lwip初始化
	{
	//if failed....
	}
//	
//	modify(&motor1, send_buff_uart);

		udp_demo_test();  		//UDP 模式

	while(1)
	{

				
	}

//	
//	
//	while(1)
//	{
//        lwip_periodic_handle();	//LWIP内核需要定时处理的函数
//        key=KEY_Scan(0);
//		if(key==KEY1_PRES)		    //按KEY1键建立连接
//		{
//			if((udp_demo_flag & 1<<5)){}// printf("UDP连接已经建立,不能重复连接\r\n");	//如果连接成功,不做任何处理
//			else udp_demo_test();		//当断开连接后,调用udp_demo_test()函数
//		}
//		delay_ms(10);
//	}  
}

