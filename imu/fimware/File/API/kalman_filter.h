#ifndef __KALMAN_FILTER__
#define __KALMAN_FILTER__

#include "main.h"
#include "gimbal_control_types.h"

void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I);
float *kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2);

#endif

