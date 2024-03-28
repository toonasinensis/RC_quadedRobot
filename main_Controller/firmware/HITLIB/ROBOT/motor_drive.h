#ifndef __MOTOR_DRIVE_H__
#define __MOTOR_DRIVE_H__

#include "hit_global_type.h"
#include "pid_algorithm.h"
//����������ṹ��
class cEncoder
{
	public:
		int32_t 	siRawValue;//���α�������ԭʼֵ
		int32_t 	siPreRawValue;//��һ�α�������ԭʼֵ
		int32_t 	siDiff;//����������ԭʼֵ�Ĳ�ֵ
		int32_t 	siSumValue;//�������ۼ�ֵ
		fp32 		  siGearRatio;//������������ٱ�
		int32_t 	siNumber;//����������
		fp32   		fpSpeed;//��������������ת�٣���λ��r/min
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
