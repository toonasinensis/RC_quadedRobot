#include "unitree_motor.h"

// clang-format off
leg_t leg[6];

//={
//								{{.command.ID = 0}, {.command.ID = 1}, {.command.ID = 2}, &huart1, GPIOA, GPIO_PIN_11},//FL,FR,RL,RR
//                {{.command.ID = 0}, {.command.ID = 1}, {.command.ID = 2}, &huart2, GPIOA, GPIO_PIN_4},
//                {{.command.ID = 0}, {.command.ID = 1}, {.command.ID = 2}, &huart3, GPIOH, GPIO_PIN_8},
//								{	{.command.ID = 0}, {.command.ID = 1}, {.command.ID = 2}, &huart6, GPIOC, GPIO_PIN_8},
//                {{.command.ID = 0}, {.command.ID = 1}, {.command.ID = 2}, &huart4, GPIOB, GPIO_PIN_11},
//								{{.command.ID = 0}, {.command.ID = 1}, {.command.ID = 2}, &huart5, GPIOD, GPIO_PIN_6}
//							};
// clang-format on

float max_kp = 2.5f, min_kd = 4.0f, max_torque = 5.0f;
uint32_t crc32_core(volatile uint8_t *src, uint32_t len) {
  uint32_t *ptr = (uint32_t *)src;
  uint32_t xbit = 0;
  uint32_t data = 0;
  uint32_t CRC32 = 0xFFFFFFFF;
  const uint32_t dwPolynomial = 0x04c11db7;
  for (uint32_t i = 0; i < len; i++) {
    xbit = 1 << 31;
    data = ptr[i];
    for (uint32_t bits = 0; bits < 32; bits++) {
      if (CRC32 & 0x80000000) {
        CRC32 <<= 1;
        CRC32 ^= dwPolynomial;
      } else
        CRC32 <<= 1;
      if (data & xbit)
        CRC32 ^= dwPolynomial;
      xbit >>= 1;
    }
  }
  return CRC32;
}

void modify(motor_send_data_t *motor_data, uint8_t *raw_data) {

  //    param_protect(motor_data,max_kp,min_kd,max_torque);
  raw_data[0] = 0xFE;
  raw_data[1] = 0xEE;
  raw_data[2] = motor_data->ID;
  raw_data[3] = 0x00;
  raw_data[4] = motor_data->mode;
  raw_data[5] = 0xff;
  raw_data[6] = 0x00;
  raw_data[7] = 0x00;
  raw_data[8] = 0x00;
  raw_data[9] = 0x00;
  raw_data[10] = 0x00;
  raw_data[11] = 0x00;
  raw_data[12] = ((int)(motor_data->T * 256) & 0x00FF);
  raw_data[13] = ((int)(motor_data->T * 256) >> 8);
  raw_data[14] = ((int)(motor_data->W * 128) & 0x00FF);
  raw_data[15] = ((int)(motor_data->W * 128) >> 8);

  raw_data[16] = ((int)(motor_data->Pos * 16384 / 2 / PI) & 0x00FF);
  raw_data[17] = ((int)(motor_data->Pos * 16384 / 2 / PI) >> 8);
  raw_data[18] = ((int)(motor_data->Pos * 16384 / 2 / PI) >> 16);
  raw_data[19] = ((int)(motor_data->Pos * 16384 / 2 / PI) >> 24);

  raw_data[20] = ((int)(motor_data->K_P * 2048) & 0x00FF);
  raw_data[21] = ((int)(motor_data->K_P * 2048) >> 8);

  raw_data[22] = ((int)(motor_data->K_W * 1024) & 0x00FF);
  raw_data[23] = ((int)(motor_data->K_W * 1024) >> 8);

  raw_data[24] = 0x00;
  raw_data[25] = 0x00;
  raw_data[26] = 0x00;
  raw_data[27] = 0x00;
  raw_data[28] = 0x00;
  raw_data[29] = 0x00;
  uint32_t crc = crc32_core(raw_data, 7);
  raw_data[30] = (uint8_t)(crc)&0xFF;
  raw_data[31] = (uint8_t)(crc >> 8);
  raw_data[32] = (uint8_t)(crc >> 16);
  raw_data[33] = (uint8_t)(crc >> 24);
}

uint32_t extract(motor_receive_data_t *motor_data, uint8_t *raw_data) {
  uint32_t crc = (raw_data[74] << 0) + (raw_data[75] << 8) +
                 (raw_data[76] << 16) + (raw_data[77] << 24);
  if (crc == crc32_core(raw_data, 18)) {
    motor_data->ID = raw_data[2];
    motor_data->mode = raw_data[4];
    motor_data->Temp = raw_data[6];
    motor_data->Error = raw_data[7];
    motor_data->T = (*(short *)(&raw_data[12])) / 256.0;
    motor_data->W = (*(short *)(&raw_data[14])) / 128.0;
    //        motor_data->Acc = (*(short *)(&raw_data[26]));
    motor_data->Pos = (*(int *)(&raw_data[30])) * 2 * PI / 16384;
    //        motor_data->gyro[0] = (*(short *)(&raw_data[38])) * 2000 * 2 * PI
    //        / 32768 / 360; motor_data->gyro[1] = (*(short *)(&raw_data[40])) *
    //        2000 * 2 * PI / 32768 / 360; motor_data->gyro[2] = (*(short
    //        *)(&raw_data[42])) * 2000 * 2 * PI / 32768 / 360;
    //        motor_data->acc[0] = (*(short *)(&raw_data[44])) * 8 * 9.80665 /
    //        32768; motor_data->acc[1] = (*(short *)(&raw_data[46])) * 8
    //        * 9.80665 / 32768; motor_data->acc[2] = (*(short
    //        *)(&raw_data[48])) * 8.0 * 9.80665 / 32768;
    return 1;
  }
  return 0;
}

uint32_t extract_new(motor_receive_data_t *motor_data, uint8_t *raw_data) {
	
	
	//if(raw_data[0]==0XFE||raw_data[1]==OXEE)
  uint8_t if_find_frame_head=0;
	int j=0;
	for( j=0;j<77;j++)
	{
		if(raw_data[j]==0xFE||raw_data[j+1]==0XEE)
		{
			if_find_frame_head = 1;
		}
		if(if_find_frame_head==1)
		{
			break;
		}
			
	}

    motor_data->ID = raw_data[2+j];
    motor_data->mode = raw_data[4+j];
    motor_data->Temp = raw_data[6+j];
    motor_data->Error = raw_data[7+j];
    motor_data->T = (*(short *)(&raw_data[12+j])) / 256.0;
    motor_data->W = (*(short *)(&raw_data[14+j])) / 128.0;
    //        motor_data->Acc = (*(short *)(&raw_data[26]));
    motor_data->Pos = (*(int *)(&raw_data[30+j])) * 2 * PI / 16384;

    return 1;
//  }
//  return 0;
}

void param_protect(motor_send_data_t *check_data, float max_kp, float min_kd,
                   float max_torque) {
  if (check_data->K_P > max_kp)
    check_data->K_P = max_kp;

  if (check_data->K_W < min_kd)
    check_data->K_W = min_kd;

  if (check_data->T > max_torque)
    check_data->T = max_torque;

  if (check_data->T < -max_torque)
    check_data->T = -max_torque;
}
