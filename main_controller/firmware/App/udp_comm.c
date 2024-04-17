
/* Includes ------------------------------------------------------------------*/
#include "udp_comm.h"
#include "system_monitor.h"

udp_receive_data_t udp_receive_data = {.header = {0xFE, 0xEE}};
udp_send_data_t udp_send_data = {.header = {0xFE, 0xEE}, .state = SYS_IDLE};

void udp_motor_type2raw_motor_type(udp_motor_send_t *udp_motor_send,
                                   motor_send_data_t *raw_motor_send) {
  // raw_motor_send->mode = udp_motor_send->mode;
  raw_motor_send->K_P = udp_motor_send->K_P;
  raw_motor_send->K_W = udp_motor_send->K_W;
  raw_motor_send->Pos = udp_motor_send->Pos;
  raw_motor_send->W = udp_motor_send->W;
  raw_motor_send->T = udp_motor_send->T;
}

void raw_motor_type2udp_motor_type(udp_motor_receive_t *udp_motor_receive,
                                   motor_receive_data_t *raw_motor_receive) {
  udp_motor_receive->Pos = raw_motor_receive->Pos;
  udp_motor_receive->W = raw_motor_receive->W;
  udp_motor_receive->Acc = raw_motor_receive->Acc;
  udp_motor_receive->T = raw_motor_receive->T;
  udp_motor_receive->Temp = raw_motor_receive->Temp;
}
