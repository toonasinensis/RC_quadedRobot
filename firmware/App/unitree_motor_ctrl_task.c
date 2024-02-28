#include "unitree_motor_ctrl_task.h"
#include "motor_control.h"

void moverecvtoleg(motor_receive_data_t *leg_recv, MOTOR_recv* go1_rcv)
{
	leg_recv->ID = go1_rcv->motor_id;
	leg_recv->mode = go1_rcv->mode;
	leg_recv->Pos = go1_rcv->Pos;
	leg_recv->W = go1_rcv->W;
	leg_recv->Error = go1_rcv->MError;
	leg_recv->T = go1_rcv->T;
}
void receive_go1motor_feedback(uint8_t *raw_data, leg_t *leg) {
  MOTOR_recv temp_receive_data;
  if (extract_data(&temp_receive_data, raw_data)) {
  
    if (temp_receive_data.motor_id == 0) {
      ++leg->hip_motor.temp_rate;
   //   leg->hip_motor.feedback.ID = temp_receive_data.motor_id;
			moverecvtoleg(&leg->hip_motor.feedback,&temp_receive_data);
    } else if (temp_receive_data.motor_id == 1) {
      ++leg->thigh_motor.temp_rate;
			moverecvtoleg(&leg->thigh_motor.feedback,&temp_receive_data);
    } else if (temp_receive_data.motor_id == 2) {
      ++leg->knee_motor.temp_rate;
			moverecvtoleg(&leg->knee_motor.feedback,&temp_receive_data);
    }
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
															 

void movelegtosend(motor_send_data_t *leg_send, MOTOR_send* go1_send)
{
go1_send->id = leg_send->ID;
	go1_send->K_P = leg_send->K_P;
		go1_send->K_W = leg_send->K_W;
go1_send->mode = leg_send->mode;
	go1_send->Pos = leg_send->Pos;
	go1_send->W = leg_send->W;
	go1_send->T = leg_send->T;
}


void send_single_go1motor_command(uint8_t *raw_data, leg_t *leg,
                               uint8_t motor_num) {
	MOTOR_send send_temp;

  switch (motor_num) {
  case 0:
	movelegtosend(	&leg->thigh_motor.command,&send_temp);
		SendGO1_Motor_Data(&send_temp,raw_data);
   
    break;
  case 1:
 	movelegtosend(	&leg->hip_motor.command,&send_temp);
		SendGO1_Motor_Data(&send_temp,raw_data);
   
    break;
  case 2:
  	movelegtosend(	&leg->knee_motor.command,&send_temp);
		SendGO1_Motor_Data(&send_temp,raw_data);
   
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
      send_single_go1motor_command(tx_raw_data[j], &leg[j], i);
    }
    for (int j = 0; j < LEG_NUM; j++) {
      receive_go1motor_feedback(rx_raw_data[j], &leg[j]);
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
