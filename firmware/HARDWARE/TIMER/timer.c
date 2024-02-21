#include "timer.h"
#include "led.h"
//////////////////////////////////////////////////////////////////////////////////
#include "system_monitor.h"
#include "udp_comm.h"
#include "unitree_motor_ctrl_task.h"

TIM_HandleTypeDef TIM3_Handler; // 瀹氭椂鍣ㄥ彞鏌�

extern u32 lwip_localtime; // lwip鏈湴鏃堕棿璁℃暟鍣�,鍗曚綅:ms
// 閫氱敤瀹氭椂鍣�3涓柇鍒濆鍖�,瀹氭椂鍣�3鍦ˋPB1涓婏紝APB1鐨勫畾鏃跺櫒鏃堕挓涓�200MHz
// arr锛氳嚜鍔ㄩ噸瑁呭€笺€�
// psc锛氭椂閽熼鍒嗛鏁�
// 瀹氭椂鍣ㄦ孩鍑烘椂闂磋绠楁柟娉�:Tout=((arr+1)*(psc+1))/Ft us.
// Ft=瀹氭椂鍣ㄥ伐浣滈鐜�,鍗曚綅:Mhz
// 杩欓噷浣跨敤鐨勬槸瀹氭椂鍣�3!(瀹氭椂鍣�3鎸傚湪APB1涓婏紝鏃堕挓涓篐CLK/2)
void TIM3_Init(u16 arr, u16 psc) {
  TIM3_Handler.Instance = TIM3;                             // 閫氱敤瀹氭椂鍣�3
  TIM3_Handler.Init.Prescaler = psc;                        // 鍒嗛
  TIM3_Handler.Init.CounterMode = TIM_COUNTERMODE_UP;       // 鍚戜笂璁℃暟鍣�
  TIM3_Handler.Init.Period = arr;                           // 鑷姩瑁呰浇鍊�
  TIM3_Handler.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // 鏃堕挓鍒嗛鍥犲瓙
  HAL_TIM_Base_Init(&TIM3_Handler);

  HAL_TIM_Base_Start_IT(
      &TIM3_Handler); // 浣胯兘瀹氭椂鍣�3鍜屽畾鏃跺櫒3鏇存柊涓柇锛歍IM_IT_UPDATE
}

// 瀹氭椂鍣ㄥ簳鍐岄┍鍔紝寮€鍚椂閽燂紝璁剧疆涓柇浼樺厛绾�
// 姝ゅ嚱鏁颁細琚獺AL_TIM_Base_Init()鍑芥暟璋冪敤
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM3) {
    __HAL_RCC_TIM3_CLK_ENABLE(); // 浣胯兘TIM3鏃堕挓
    HAL_NVIC_SetPriority(TIM3_IRQn, 1,
                         3); // 璁剧疆涓柇浼樺厛绾э紝鎶㈠崰浼樺厛绾�1锛屽瓙浼樺厛绾�3
    HAL_NVIC_EnableIRQ(TIM3_IRQn); // 寮€鍚疘TM3涓柇
  }
}

// 瀹氭椂鍣�3涓柇鏈嶅姟鍑芥暟
void TIM3_IRQHandler(void) { HAL_TIM_IRQHandler(&TIM3_Handler); }
uint8_t start_send;
uint8_t if_long_use = 0;
uint8_t fast_send = 0;
// 瀹氭椂鍣�3涓柇鏈嶅姟鍑芥暟璋冪敤
uint32_t timer_pp;
uint32_t psc_1000;
extern float normal_frecquency;
extern u8 udp_send_flag;
float otto_pos;
uint16_t motor_fps[18]={0};
int test_id = 0;

extern  int udp_cnt;
int   udp_fps;
uint8_t loss_udp_detect;




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
			test_id = uart_tx_buffer[3][2];
    }

    if (psc_1000 > 1000) // 鍒嗛鍒�1s
    {
			
			udp_fps = udp_cnt;
			if(udp_cnt<200)
			{
				loss_udp_detect++;
				if(loss_udp_detect>1)
				{
					loss_udp_detect = 0;
					//HAL_NVIC_SystemReset();
				}
			}
			udp_cnt = 0;
      normal_frecquency = 0;
      //cal_fps_sys(&system_monitor);
      psc_1000 = 0;
      for (int i = 0; i < LEG_NUM; i++) {
        leg[i].hip_motor.real_rate   = leg[i].hip_motor.temp_rate;
        leg[i].thigh_motor.real_rate = leg[i].thigh_motor.temp_rate;
        leg[i].knee_motor.real_rate  = leg[i].knee_motor.temp_rate;
				motor_fps[3*i] = leg[i].hip_motor.real_rate;
				motor_fps[3*i+1] = leg[i].thigh_motor.real_rate;
				motor_fps[3*i+2] = leg[i].knee_motor.real_rate;
        leg[i].hip_motor.temp_rate = 0;
        leg[i].thigh_motor.temp_rate = 0;
        leg[i].knee_motor.temp_rate = 0;
      }
    }

    // 璁剧疆udp鍙戦€佹暟鎹細
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
    lwip_localtime += 1; // 鍔�10
  }
}
