#include "remote_lcd_task.h"
#include "navigation_task.h"
#include "locate_algorithm.h"

extern int16_t M_SPEED;
extern float q_correct;

static void wifi_rxdata_update(void)
{
	uart4_tx_buf[0].float_num = system_state;
	uart4_tx_buf[1].float_num = 1.0;//system_error;

	uart4_tx_buf[2].float_num = 0.0;//keyboard_mode;
	uart4_tx_buf[3].float_num = nav.state;//nav.state;
	uart4_tx_buf[4].float_num = 0.0;//reloc_times;

	uart4_tx_buf[5].float_num = 0.0;//q_correct; // wifi_fps

	uart4_tx_buf[6].float_num = stRobot.stPot.fpPosX;
	uart4_tx_buf[7].float_num = stRobot.stPot.fpPosY;
	uart4_tx_buf[8].float_num = stRobot.stPot.fpPosQ;

	uart4_tx_buf[9].float_num = 9.0;//point_end.x_end;  
	uart4_tx_buf[10].float_num = 10.0;//point_end.y_end; 
	uart4_tx_buf[11].float_num = 11.0;//point_end.q_end;

	uart4_tx_buf[12].float_num = 12.0;//flag_color; 
	uart4_tx_buf[13].float_num = 13.0;//goal_num;	
	uart4_tx_buf[14].float_num = 14.0;//flag_auto_fetch;			

	uart4_tx_buf[15].float_num = stFollowerWheel.siCoderACur;
	uart4_tx_buf[16].float_num = stFollowerWheel.siCoderBCur;
	uart4_tx_buf[17].float_num = stGyro.fpQ_Cur; 

	uart4_tx_buf[18].float_num = 18.0;//dt35_x1;
	uart4_tx_buf[19].float_num = 19.0;//dt35_x2;
	uart4_tx_buf[20].float_num = 20.0;//dt35_y1;
	uart4_tx_buf[21].float_num = 21.0;//dt35_y2;

  memcpy(&uart4_tx_data_buf[4], uart4_tx_buf, 4 * 26 * sizeof(uint8_t)); //要随着上面发送的变量数量改变
  uart4_tx_data_buf[0] = usart4_eft.start1;
	uart4_tx_data_buf[1] = usart4_eft.start2;
	uart4_tx_data_buf[2] = usart4_eft.ID;
	uart4_tx_data_buf[3] = usart4_eft.datanum;
  uart4_tx_data_buf[118] = usart4_eft.tail1;
	uart4_tx_data_buf[119] = usart4_eft.tail2;
	
	
	
	for(int i = 0;i<40;i++)
	{
	 uart4_tx_buf[15].float_num = i;
	}	
	

}

void lcd_display(void)
{
	wifi_rxdata_update();
//	memcpy(&uart4_tx_data_buf[4], usart4_eft.num,REMOTE_TX_DATA_LEN); //要随着上面发送的变量数量改变
//	USART6_DMA_Tx(REMOTE_CTRL);
	HAL_UART_Transmit_DMA (&huart4,uart4_tx_data_buf,REMOTE_TX_LEN);
	
  data_monitor.remote_TX.cnt++;
}

void vision_send_data(void)
{
	uart5_tx_buf[0].float_num = stRobot.stPot.fpPosX;
	uart5_tx_buf[1].float_num = stRobot.stPot.fpPosY;
	uart5_tx_buf[2].float_num = stRobot.stPot.fpPosQ;
	uart5_tx_buf[3].float_num = 36;
	
	memcpy(&uart5_tx_data_buf[3], uart5_tx_buf, 4 * 4 * sizeof(uint8_t)); //要随着上面发送的变量数量改变
  uart5_tx_data_buf[0] = usart5_eft.start1;
	uart5_tx_data_buf[1] = usart5_eft.start2;
	uart5_tx_data_buf[2] = usart5_eft.datanum;
  uart5_tx_data_buf[19] = usart5_eft.tail1;
	uart5_tx_data_buf[20] = usart5_eft.tail2;

	HAL_UART_Transmit_DMA (&huart5,uart5_tx_data_buf,VISION_TX_LEN);
	data_monitor.vision_TX.cnt++;	
	
}