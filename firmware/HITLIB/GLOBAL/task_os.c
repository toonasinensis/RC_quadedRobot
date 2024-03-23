#include "task_os.h"
#include "motor_control_task.h"
#include "navigation_task.h"
#include "locate_algorithm.h"
#include "remote_lcd_task.h"
#include "parallel_mechanism_task.h"
#include "vision_task.h"
#include "unitree_motor_ctrl_task.h"
//extern void chassis_control(void);
#include "udp_basic.h"
#include "udp_comm.h"

uint8_t start_send;
uint8_t if_long_use = 0;
int timer_psc;
uint8_t fast_send = 0;
void motor_ctrl_Task(void)
{
	if (start_send || if_long_use || fast_send) {
      if (if_long_use && timer_psc > 1000) {
        timer_psc = 0;
        start_send = 0;
				
        send_all_motor_command(uart_tx_buffer, uart_rx_buffer, leg);
      } else if (!if_long_use) {
        send_all_motor_command(uart_tx_buffer, uart_rx_buffer, leg);
      }
      timer_psc++;

    }
	
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
	
	system_monitor.motor_crl_Task_cnt++;
}

uint8_t udp_buff_test[200];

void udp_send_task(void)
{

}


	
void action_task(void)
{
  vision_process();
	
	system_monitor.action_task_cnt++;
}


uint8_t remote_crl_cnt;
void remote_ctrl_task(void)
{
//	Js_Deal();
//	Key_Deal();
//	
//	if(flag_auto_fetch_all)
//		{
//			Auto_Key_Deal();
//		}

//		if(remote_crl_cnt == 2 || remote_crl_cnt ==4)
//		{
//		vision_send_data();
//		}
//	
//		
//		if(remote_crl_cnt ==4)
//		{
//	   lcd_display();
//		 remote_crl_cnt = 0;
//		}
//		

//		remote_crl_cnt++;
//	system_monitor.remote_ctrl_task_cnt++;
}
void SystemMonitorTask(void)
{
	system_monitor.System_cnt++;
	if(system_monitor.System_cnt == 1000)
	{
		//UDP_SendData(udp_buff_test,200);
	  cal_fps_sys(&system_monitor);	
    data_monitor.RU_run_motor.fps = data_monitor.RU_run_motor.cnt;
    data_monitor.RD_run_motor.fps = data_monitor.RD_run_motor.cnt;
    data_monitor.LU_run_motor.fps = data_monitor.LU_run_motor.cnt;
    data_monitor.LD_run_motor.fps = data_monitor.LD_run_motor.cnt;
    data_monitor.FollowerWheel.fps = data_monitor.FollowerWheel.cnt;
    data_monitor.gyro_yaw.fps = data_monitor.gyro_yaw.cnt;
		data_monitor.remote_RX.fps = data_monitor.remote_RX.cnt;
		data_monitor.remote_TX.fps = data_monitor.remote_TX.cnt;
		data_monitor.vision_RX.fps = data_monitor.vision_RX.cnt;
		data_monitor.vision_TX.fps = data_monitor.vision_TX.cnt;
		
    PM_Motor.up_motor.fb_data.receive_rate = PM_Motor.up_motor.fb_data.receive_cnt;
    PM_Motor.rightdown_motor.fb_data.receive_rate = PM_Motor.rightdown_motor.fb_data.receive_cnt;		
    PM_Motor.leftdown_motor.fb_data.receive_rate = PM_Motor.leftdown_motor.fb_data.receive_cnt;
    PM_Motor.up_motor.crl_data.send_rate = PM_Motor.up_motor.crl_data.send_cnt;
    PM_Motor.rightdown_motor.crl_data.send_rate = PM_Motor.rightdown_motor.crl_data.send_cnt;		
    PM_Motor.leftdown_motor.crl_data.send_rate = PM_Motor.leftdown_motor.crl_data.send_cnt;

		data_monitor.RU_run_motor.cnt = 0;
		data_monitor.RD_run_motor.cnt = 0;
		data_monitor.LU_run_motor.cnt = 0;
		data_monitor.LD_run_motor.cnt = 0;
		data_monitor.FollowerWheel.cnt = 0;
		data_monitor.gyro_yaw.cnt = 0;
		data_monitor.remote_RX.cnt = 0;
		data_monitor.remote_TX.cnt = 0;
		data_monitor.vision_RX.cnt = 0;
		data_monitor.vision_TX.cnt = 0;
		
		PM_Motor.up_motor.fb_data.receive_cnt = 0;
		PM_Motor.rightdown_motor.fb_data.receive_cnt = 0;
		PM_Motor.leftdown_motor.fb_data.receive_cnt = 0;
		PM_Motor.up_motor.crl_data.send_cnt = 0;
		PM_Motor.rightdown_motor.crl_data.send_cnt = 0;
		PM_Motor.leftdown_motor.crl_data.send_cnt = 0;		
	}
	 //自动路径计数器
	 nav.auto_path.run_time++;

}
//void gait_task(void)
//{
//}
//void DebugTask(void)
//{
//}	

