#include "main.h"

/*--------------------------------------
任务功能：主函数
--------------------------------------*/
int main(void)
{
    BSP_Init(); //外设初始化
    OS_RUN();   //执行各项任务
}
/*--------------------------------------
任务功能：系统监视器
--------------------------------------*/
void SystemMonitorTask(void)
{
    SystemErrorDetect();

}
/*---------------------------------------------
任务功能：中断服务任务，处理费时的中断任务
注意事项：IRQ任务较多，分配堆栈应该大一些
----------------------------------------------*/
void IRQHandlerTask(void)
{

}
/*--------------------------------------
任务功能：IMU采样
--------------------------------------*/
void IMUSampleTask(void)
{
    Sensor_Data_Prepare();
    system_monitor.IMUSampleTask_cnt++;
}
/*--------------------------------------
任务功能：IMU数据更新
--------------------------------------*/
void IMUUpdateTask(void)
{
    /*设置重力互补融合修正kp系数*/
    imu_data.gkp = 0.4f;

    /*设置重力互补融合修正ki系数*/
    imu_data.gki =  0.002f;

    IMU_Update_Mahony(&imu_data,1e-3f);	//1ms执行一次
    system_monitor.IMUUpdateTask_cnt++;
}
/*--------------------------------------
任务功能：温度控制
--------------------------------------*/
void TempControlTask(void)
{
    Temp_Control();
    system_monitor.TempControlTask_cnt++;
}
/*--------------------------------------
任务功能：云台任务
--------------------------------------*/
int test = 1;
void GimbalTask(void)
{
   // Servo_Control((ServoState)G_ST_IMU.Receive.ReloadStatus);
//		Servo_Control((ServoState)test);
    system_monitor.GimbalTask_cnt++;
}

/*--------------------------------------
任务功能：数据发送
--------------------------------------*/
void SendDataTask(void)
{
    static unsigned int cnt = 0;
    cnt++;
    if(cnt > 1)
    {
			MainControl_Tx_Protocol();
		//	Camera_Tx_Protocol();
      system_monitor.SendDataTask_cnt++;
    }
}
/*--------------------------------------
任务功能：LED任务
--------------------------------------*/
void LedTask(void)
{
    LED_BLUE_ON();
    LED_GREEN_TOGGLE();
    system_monitor.LedTask_cnt++;
}
/*--------------------------------------
任务功能：在线调试、上传自定义数据
--------------------------------------*/
void DebugTask(void)
{
}
