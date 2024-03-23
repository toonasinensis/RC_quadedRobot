#ifndef __MOTOR_DRIVE_H__
#define __MOTOR_DRIVE_H__

#include "hit_global_type.h"
#include "pid_algorithm.h"
//电机编码器结构体
class cEncoder
{
	public:
		int32_t 	siRawValue;//本次编码器的原始值
		int32_t 	siPreRawValue;//上一次编码器的原始值
		int32_t 	siDiff;//编码器两次原始值的差值
		int32_t 	siSumValue;//编码器累加值
		fp32 		  siGearRatio;//电机减速器减速比
		int32_t 	siNumber;//编码器线数
		fp32   		fpSpeed;//电机减速器输出轴转速，单位：r/min
		cEncoder(){}
		cEncoder( fp32 gr, int32_t num)
		{
			this->siNumber = num;
			this->siGearRatio = gr;
		}
	void Encoder_Process(int32_t value,uint8_t type);
	void GetPosition(void);
};

class cMotor
{
	public:
	int32_t real_current;
	int32_t feed_forward_current;
	cPID velt_pid;
	cPID pos_pid;
	cEncoder encoder;
	cMotor(){}
	cMotor(fp32 vKp, fp32 vKi, fp32 vKd, fp32 vUpMax, fp32 vEiMax, fp32 vUdMax, fp32 vEMin,
				 fp32 pKp, fp32 pKi, fp32 pKd, fp32 pUpMax, fp32 pEiMax, fp32 pUdMax, fp32 pEMin,
				 fp32 gr, int32_t num,fp32 ts)
	{
		velt_pid = cPID(vKp, vKi, vKd, vUpMax, vEiMax, vUdMax, vEMin, ts);
		pos_pid  = cPID(pKp, pKi, pKd, pUpMax, pEiMax, pUdMax, pEMin, ts);
		encoder  = cEncoder(gr, num);
	}
};



//int32_t Get_Encoder_Number(CanRxMsg* rx_message);
//int32_t Get_Speed(CanRxMsg* rx_message);
//void can_send_data(CAN_TypeDef* CANx, uint32_t StdID, int16_t ssMotor1, int16_t ssMotor2, 
//															int16_t ssMotor3, int16_t ssMotor4);
#endif
