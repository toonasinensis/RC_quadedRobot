#ifndef __DISTANCE_MEASURE_TASK_H__
#define __DISTANCE_MEASURE_TASK_H__

#include "stm32f4xx.h"
#include "stdio.h"
#include "global_declare.h"

void UA1_Configuration(void);
void UA2_Configuration(void);
void UA3_Configuration(void);
void UART3_Configuration(void);

extern char DistanceMeasureUA1Buf[];
extern char DistanceMeasureUA2Buf[];
extern char DistanceMeasureUA3Buf[];

extern float X_Axis_Distance;
extern float Y_Axis_Distance;
extern float Z_Axis_Distance;


#endif
