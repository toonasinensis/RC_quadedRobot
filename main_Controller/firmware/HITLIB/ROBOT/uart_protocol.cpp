#include "uart_protocol.h"
#include "system_monitor.h"

//蓝牙串口
uart4_tx_protocol_t usart4_eft = 
{
	0x55, 
	0x22,
	REMOTE_CTRL,
	REMOTE_TX_DATA_LEN,
	{0},
	0xBB,
	0xAA
};

uart4_rx_protocol_t usart4_efr = 
{
  0x55, 
	0x22,
	REMOTE_CTRL,
	REMOTE_RX_DATA_LEN,
	{0},
	0xBB,
	0xAA 
};

union float2char uart4_tx_buf[40];


uint8_t usart4_efr_dma[REMOTE_RX_LEN] = {0};

/*串口1相关的缓存区*/
uint8_t uart4_tx_data_buf[REMOTE_TX_LEN] = {0};
uint8_t remote_rx_flag = 0;
void Comm4Rx_IRQ(void)//串口4电流DMA接收函数
{
		/*********************************状态机解析数据包************************************/
	if(usart4_efr_dma[0] == usart4_efr.start1 && usart4_efr_dma[1] == usart4_efr.start2)
	{
	 if(usart4_efr_dma[37] == usart4_efr.tail1 && usart4_efr_dma[38] == usart4_efr.tail2)
	 {
	 if(usart4_efr_dma[2] == usart4_efr.ID && usart4_efr_dma[3] == usart4_efr.datanum)
	  {
				memcpy(usart4_efr.num, &usart4_efr_dma[4] , REMOTE_RX_DATA_LEN);
        data_monitor.remote_RX.cnt++;
			  remote_rx_flag = 1;
	  }	
	 }	
	}
}

//视觉串口
uart5_tx_protocol_t usart5_eft = 
{
	0x55, 
	0x22,
	VISION_TX_DATA_LEN,
	{0},
	0xBB,
	0xAA
};

uart5_rx_protocol_t usart5_efr = 
{
  0x55, 
	0x00,
	VISION_RX_DATA_LEN,
	{0},
	0x00,
	0xAA 
};

union float2char uart5_tx_buf[4];
uint8_t uart5_tx_data_buf[VISION_TX_LEN];
uint8_t usart5_efr_dma[VISION_RX_LEN] = {0};

uint8_t vision_rx_flag = 0;
union float2char vision_data_buff[3] ;
VISION_DATA vision_data;
fp32 vision_dead_zone = 25;
//fp32 vision_x_fb_com = 0;
//fp32 vision_y_fb_com = 0;
void Comm5Rx_IRQ(void)//串口5电流DMA接收函数
{
		/*********************************状态机解析数据包************************************/
	if(usart5_efr_dma[0] == usart5_efr.start1 && usart5_efr_dma[1] == usart5_efr.start2)
	{
	 if(usart5_efr_dma[15] == usart5_efr.tail1 && usart5_efr_dma[16] == usart5_efr.tail2)
	 {
	 if( usart5_efr_dma[2] == usart5_efr.datanum)
	  {
				memcpy(usart5_efr.num, &usart5_efr_dma[3] , VISION_RX_DATA_LEN);
        vision_data_buff[0].char_num[0] = usart5_efr.num[0];
        vision_data_buff[0].char_num[1] = usart5_efr.num[1];			
        vision_data_buff[0].char_num[2] = usart5_efr.num[2];
        vision_data_buff[0].char_num[3] = usart5_efr.num[3];	
        vision_data_buff[1].char_num[0] = usart5_efr.num[4];
        vision_data_buff[1].char_num[1] = usart5_efr.num[5];			
        vision_data_buff[1].char_num[2] = usart5_efr.num[6];
        vision_data_buff[1].char_num[3] = usart5_efr.num[7];	
        vision_data_buff[2].char_num[0] = usart5_efr.num[8];
        vision_data_buff[2].char_num[1] = usart5_efr.num[9];			
        vision_data_buff[2].char_num[2] = usart5_efr.num[10];
        vision_data_buff[2].char_num[3] = usart5_efr.num[11];				
			  
			  vision_data.x =  vision_data_buff[0].float_num * 1000 ;//单位转为mm
			  vision_data.y =  vision_data_buff[1].float_num * 1000 ;
			  if(vision_data_buff[2].float_num != 0)
				{
  			vision_data.z =  vision_data_buff[2].float_num * 1000;				
				}

   			if(fabs(vision_data.x ) < vision_dead_zone)
				{
					vision_data.x = 0;
				}					
	      if(fabs(vision_data.y) < vision_dead_zone)
				{
					vision_data.y = 0;
				}
//	      if(fabs(vision_data.z) < vision_dead_zone)
//				{
//					vision_data.z = 0;
//				}
 			
  		  
        data_monitor.vision_RX.cnt++;
			  vision_rx_flag = 1;
	  }	
	 }	
	}
}

