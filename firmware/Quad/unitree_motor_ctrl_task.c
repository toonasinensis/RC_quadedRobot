#include "unitree_motor_ctrl_task.h"

#include "udp_comm.h"




void init_leg_motor(void)
{
	
	
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_SET);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	
	//eth reset
	  GPIO_InitStruct.Pin = GPIO_PIN_2;	
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
	
	
	

	//rs485 control direction
	  GPIO_InitStruct.Pin = GPIO_PIN_11;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);//rs485 control direction
	  GPIO_InitStruct.Pin = GPIO_PIN_11;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);	//rs485 control direction
	  GPIO_InitStruct.Pin = GPIO_PIN_4;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);	//rs485 control direction
	  GPIO_InitStruct.Pin = GPIO_PIN_8;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);	//rs485 control direction
	  GPIO_InitStruct.Pin = GPIO_PIN_6;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);	//rs485 control direction
	  GPIO_InitStruct.Pin = GPIO_PIN_8;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	udp_receive_data.header[0] = 0xFE;
		udp_receive_data.header[1] = 0xEE;
	
	udp_send_data.header[0] = 0xFE;
		udp_send_data.header[1] = 0xEE;
	
	udp_send_data.state = 0;

	leg[0].en_gpio_pin =  GPIO_PIN_11;
	leg[0].en_gpio_port = GPIOA;
	leg[0].p_huart = &huart1;
	leg[0].hip_motor.command.ID = 0;
	leg[0].thigh_motor.command.ID = 1;
	leg[0].knee_motor.command.ID = 2;

	leg[1].en_gpio_pin =  GPIO_PIN_4;
	leg[1].en_gpio_port = GPIOA;
	leg[1].p_huart = &huart2;

	leg[2].en_gpio_pin =  GPIO_PIN_8;
	leg[2].en_gpio_port = GPIOH;
	leg[2].p_huart = &huart3;


   leg[3].en_gpio_pin =  GPIO_PIN_8;
	leg[3].en_gpio_port = GPIOC;
	leg[3].p_huart = &huart6;

	for(int i=0;i<LEG_NUM;i++)
{
	leg[i].hip_motor.command.ID = 0;
	leg[i].thigh_motor.command.ID = 1;
	leg[i].knee_motor.command.ID = 2;
}
}







void receive_motor_feedback(uint8_t *raw_data, leg_t *leg) {
  motor_receive_data_t temp_receive_data;
  // while(leg->p_huart->RxState != HAL_UART_STATE_READY){};
  if (extract_new(&temp_receive_data, raw_data)) {
    if (temp_receive_data.Error != 0) {
      // enter all_stop mode
    }
    if (temp_receive_data.ID == 0) {
      ++leg->hip_motor.temp_rate;
      leg->hip_motor.feedback = temp_receive_data;
    } else if (temp_receive_data.ID == 1) {
      ++leg->thigh_motor.temp_rate;
      leg->thigh_motor.feedback = temp_receive_data;
    } else if (temp_receive_data.ID == 2) {
      ++leg->knee_motor.temp_rate;
      leg->knee_motor.feedback = temp_receive_data;
    }
  }
}

void send_single_motor_command(uint8_t *raw_data, leg_t *leg,
                               uint8_t motor_num) {

  switch (motor_num) {
  case 0:
    modify(&leg->hip_motor.command, raw_data);
//    leg->hip_motor.cnt++;
    break;
  case 1:
    modify(&leg->thigh_motor.command, raw_data);
//    leg->thigh_motor.cnt++;
    break;
  case 2:
    modify(&leg->knee_motor.command, raw_data);
  //  leg->knee_motor.cnt++;
    break;
  }

   HAL_GPIO_WritePin(leg->en_gpio_port,leg->en_gpio_pin,GPIO_PIN_SET);
  HAL_UART_Transmit_DMA(leg->p_huart, raw_data, UART_TX_LEN);
  while (leg->p_huart->gState != HAL_UART_STATE_READY) {
  };
  	HAL_GPIO_WritePin(leg->en_gpio_port,leg->en_gpio_pin,GPIO_PIN_RESET);
}
void send_all_motor_command(uint8_t tx_raw_data[][UART_TX_LEN],
                            uint8_t rx_raw_data[][UART_RX_LEN], leg_t *leg) {
  for (int i = 0; i < JOINT_NUM; i++) {
    for (int j = 0; j < LEG_NUM; j++) {
			
			SCB_CleanDCache_by_Addr((uint32_t*)tx_raw_data,8*UART_TX_LEN);
      send_single_motor_command(tx_raw_data[j], &leg[j], i);
    }
    for (int j = 0; j < LEG_NUM; j++) {
      receive_motor_feedback(rx_raw_data[j], &leg[j]);
    }
  }
}

void disable_all_motor(void) {
  for (int i = 0; i < 4; ++i) {
    leg[i].thigh_motor.command.mode = DISABLE_MODE;
    leg[i].hip_motor.command.mode = DISABLE_MODE;
    leg[i].knee_motor.command.mode = DISABLE_MODE;
  }
}

void enable_all_motor(void) {
  for (int i = 0; i < 4; ++i) {
    leg[i].thigh_motor.command.mode = ENABLE_MODE;
    leg[i].hip_motor.command.mode = ENABLE_MODE;
    leg[i].knee_motor.command.mode = ENABLE_MODE;
  }
}

void update_command(void) {

}
// 0 是正常状态 //64 掉电 //4 堵转，过热
void A1_protect(void) {
  // 电机报错就失能
  for (int i = 0; i < 4; i++) {
    if (leg[i].knee_motor.feedback.Error != 0 ||
        leg[i].thigh_motor.feedback.Error != 0) {
      disable_all_motor(); // A1电机
    }
  }
}

int send_motor_cnt = 1;
void send_command(void) {}

void get_init_pos(void) {}
