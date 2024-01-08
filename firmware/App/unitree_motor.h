#ifndef __UNITREE_MOTOR_H__
#define __UNITREE_MOTOR_H__
#include "global_declare.h"
#include "hitcrt_types.h"
#include "usart.h"


// #include "main.h"
// #include "usart.h"
// #include "delay.h"
// #include "usart.h"

#define PI 3.1415926536f

// #define LEG_NUM  6
typedef enum {
  ENABLE_MODE = 10,
  DISABLE_MODE = 0,
} motor_mode_e;

typedef struct {
  uint8_t ID;
  uint8_t mode;
  float T;
  float W;
  float Pos;
  float K_P;
  float K_W;
} motor_send_data_t;

typedef struct {
  uint8_t ID;
  uint8_t mode;
  float T;
  float Temp;
  float Error;
  float W;
  float Pos;
  float Acc;
  float K_P;
  float K_W;
  float gyro[3];
  float acc[3];
} motor_receive_data_t;

typedef struct {
  motor_send_data_t command;
  motor_receive_data_t feedback;
  uint32_t temp_rate, real_rate;
  uint32_t cnt;
  uint32_t fps;
} unitree_motor_t;

typedef struct {

  unitree_motor_t hip_motor, thigh_motor, knee_motor;
  // 485 related
  UART_HandleTypeDef *p_huart;
  GPIO_TypeDef *en_gpio_port;
  uint16_t en_gpio_pin;

} leg_t;

extern void param_protect(motor_send_data_t *check_data, float max_kp,
                          float min_kd, float max_torque);

extern uint32_t crc32_core(volatile uint8_t *src, uint32_t len);
extern uint32_t extract(motor_receive_data_t *motor_data, uint8_t *raw_data);
extern void modify(motor_send_data_t *motor_data, uint8_t *raw_data);

extern leg_t leg[6];

#endif // __UNITREE_MOTOR_H__
