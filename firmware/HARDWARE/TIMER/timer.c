#include "timer.h"
#include "led.h"
//////////////////////////////////////////////////////////////////////////////////
#include "system_monitor.h"
#include "udp_comm.h"
#include "unitree_motor_ctrl_task.h"

TIM_HandleTypeDef TIM3_Handler; // 定时器句柄

extern u32 lwip_localtime; // lwip本地时间计数器,单位:ms
// 通用定时器3中断初始化,定时器3在APB1上，APB1的定时器时钟为200MHz
// arr：自动重装值。
// psc：时钟预分频数
// 定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
// Ft=定时器工作频率,单位:Mhz
// 这里使用的是定时器3!(定时器3挂在APB1上，时钟为HCLK/2)
void TIM3_Init(u16 arr, u16 psc) {
  TIM3_Handler.Instance = TIM3;                             // 通用定时器3
  TIM3_Handler.Init.Prescaler = psc;                        // 分频
  TIM3_Handler.Init.CounterMode = TIM_COUNTERMODE_UP;       // 向上计数器
  TIM3_Handler.Init.Period = arr;                           // 自动装载值
  TIM3_Handler.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // 时钟分频因子
  HAL_TIM_Base_Init(&TIM3_Handler);

  HAL_TIM_Base_Start_IT(
      &TIM3_Handler); // 使能定时器3和定时器3更新中断：TIM_IT_UPDATE
}

// 定时器底册驱动，开启时钟，设置中断优先级
// 此函数会被HAL_TIM_Base_Init()函数调用
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM3) {
    __HAL_RCC_TIM3_CLK_ENABLE(); // 使能TIM3时钟
    HAL_NVIC_SetPriority(TIM3_IRQn, 1,
                         3); // 设置中断优先级，抢占优先级1，子优先级3
    HAL_NVIC_EnableIRQ(TIM3_IRQn); // 开启ITM3中断
  }
}

// 定时器3中断服务函数
void TIM3_IRQHandler(void) { HAL_TIM_IRQHandler(&TIM3_Handler); }
uint8_t start_send;
uint8_t if_long_use = 0;
uint8_t fast_send = 0;
// 定时器3中断服务函数调用
uint32_t timer_pp;
uint32_t psc_1000;
extern float normal_frecquency;
extern u8 udp_send_flag;
float otto_pos;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == (&TIM3_Handler)) {
    psc_1000++;
    if (start_send || if_long_use || fast_send) {
      if (if_long_use && timer_pp > 1000) {
        timer_pp = 0;
        start_send = 0;
        send_all_motor_command(uart_tx_buffer, uart_rx_buffer, leg);
      } else if (!if_long_use) {
        send_all_motor_command(uart_tx_buffer, uart_rx_buffer, leg);
      }
      timer_pp++;
      otto_pos = leg[0].knee_motor.feedback.W;
    }

    if (psc_1000 > 1000) // 分频到1s
    {
      normal_frecquency = 0;
      //cal_fps_sys(&system_monitor);
      psc_1000 = 0;
      for (int i = 0; i < LEG_NUM; i++) {
        leg[i].hip_motor.real_rate = leg[i].hip_motor.temp_rate;
        leg[i].thigh_motor.real_rate = leg[i].thigh_motor.temp_rate;
        leg[i].knee_motor.real_rate = leg[i].knee_motor.temp_rate;
        leg[i].hip_motor.temp_rate = 0;
        leg[i].thigh_motor.temp_rate = 0;
        leg[i].knee_motor.temp_rate = 0;
      }
    }

    // 设置udp发送数据：
    // update motor feedback data
    for (int i = 0; i < LEG_NUM; ++i) {
      raw_motor_type2udp_motor_type(&udp_send_data.udp_motor_receive[i * 3],
                                    &leg[i].hip_motor.feedback);
      raw_motor_type2udp_motor_type(&udp_send_data.udp_motor_receive[i * 3 + 1],
                                    &leg[i].thigh_motor.feedback);
      raw_motor_type2udp_motor_type(&udp_send_data.udp_motor_receive[i * 3 + 2],
                                    &leg[i].knee_motor.feedback);
    }
    // add crc
    udp_send_data.check_digit =
        crc32_core((uint8_t *)&udp_send_data, sizeof(udp_send_data) / 4 - 1);
    //	udp_send_flag = 1;
    lwip_localtime += 1; // 加10
  }
}
