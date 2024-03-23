#ifndef __UDP_COMM_H__
#define __UDP_COMM_H__

#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "lwip/udp.h"
#include "main.h"
#include "unitree_motor.h"
#include <stdio.h>
#include <string.h>

typedef struct
{

			  float ang_vel_x;             //4
        float ang_vel_y;             //4
        float ang_vel_z;               //4

        float quat_X;              //4
        float quat_Y;              //4
        float quat_Z;              //4
			  float quat_W;              //4
			
			  float acc_x_send;             //4
        float acc_y_send;             //4
        float acc_z_send;               //4


}__attribute__((packed)) udp_imu_send_t;

// motor command
typedef struct
{
    // uint8_t mode;
    float T;
    float W;
    float Pos;
    float K_P;
    float K_W;
} __attribute__((packed)) udp_motor_send_t;

// motor feedback
typedef struct
{
	//	uint8_t state;
    float Pos;
    float W;
    float Acc;
    float T;
    float Temp;
} __attribute__((packed)) udp_motor_receive_t;

// send to PC throught UDP

typedef struct
{
    uint8_t header[2];
    uint8_t state;
    udp_motor_receive_t udp_motor_receive[3*LEG_NUM];
	udp_imu_send_t udp_imu;
    uint32_t check_digit;
} __attribute__((packed)) udp_send_data_t;

// receive from PC throught UDP
typedef struct
{
    uint8_t header[2];
    uint8_t state;
    udp_motor_send_t udp_motor_send[3*LEG_NUM];
    uint32_t check_digit;
} __attribute__((packed)) udp_receive_data_t;


extern void udp_motor_type2raw_motor_type(udp_motor_send_t *udp_motor_send,
                                   motor_send_data_t *raw_motor_send);
extern void raw_motor_type2udp_motor_type(udp_motor_receive_t *udp_motor_receive,
                                   motor_receive_data_t *raw_motor_receive);
extern void udp_echoserver_init(void);
extern udp_receive_data_t udp_receive_data;
extern udp_send_data_t udp_send_data;

#endif /* __UDP_COMM_H__ */
