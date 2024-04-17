#ifndef __LASER_H__
#define __LASER_H__



#include "stm32f4xx.h"



#define LASER_ON()  GPIO_SetBits(GPIOA,   GPIO_Pin_10)  //������
#define LASER_OFF() GPIO_ResetBits(GPIOA, GPIO_Pin_10)  //�ؼ���



void Laser_Configuration(void);



#endif
