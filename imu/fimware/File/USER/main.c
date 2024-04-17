#include "main.h"

/*--------------------------------------
�����ܣ�������
--------------------------------------*/
int main(void)
{
    BSP_Init(); //�����ʼ��
    OS_RUN();   //ִ�и�������
}
/*--------------------------------------
�����ܣ�ϵͳ������
--------------------------------------*/
void SystemMonitorTask(void)
{
    SystemErrorDetect();

}
/*---------------------------------------------
�����ܣ��жϷ������񣬴����ʱ���ж�����
ע�����IRQ����϶࣬�����ջӦ�ô�һЩ
----------------------------------------------*/
void IRQHandlerTask(void)
{

}
/*--------------------------------------
�����ܣ�IMU����
--------------------------------------*/
void IMUSampleTask(void)
{
    Sensor_Data_Prepare();
    system_monitor.IMUSampleTask_cnt++;
}
/*--------------------------------------
�����ܣ�IMU���ݸ���
--------------------------------------*/
void IMUUpdateTask(void)
{
    /*�������������ں�����kpϵ��*/
    imu_data.gkp = 0.4f;

    /*�������������ں�����kiϵ��*/
    imu_data.gki =  0.002f;

    IMU_Update_Mahony(&imu_data,1e-3f);	//1msִ��һ��
    system_monitor.IMUUpdateTask_cnt++;
}
/*--------------------------------------
�����ܣ��¶ȿ���
--------------------------------------*/
void TempControlTask(void)
{
    Temp_Control();
    system_monitor.TempControlTask_cnt++;
}
/*--------------------------------------
�����ܣ���̨����
--------------------------------------*/
int test = 1;
void GimbalTask(void)
{
   // Servo_Control((ServoState)G_ST_IMU.Receive.ReloadStatus);
//		Servo_Control((ServoState)test);
    system_monitor.GimbalTask_cnt++;
}

/*--------------------------------------
�����ܣ����ݷ���
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
�����ܣ�LED����
--------------------------------------*/
void LedTask(void)
{
    LED_BLUE_ON();
    LED_GREEN_TOGGLE();
    system_monitor.LedTask_cnt++;
}
/*--------------------------------------
�����ܣ����ߵ��ԡ��ϴ��Զ�������
--------------------------------------*/
void DebugTask(void)
{
}
