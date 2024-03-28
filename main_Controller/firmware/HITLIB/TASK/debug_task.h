#ifndef __DEBUG_TASK__
#define __DEBUG_TASK__

#include "hit_global_type.h"



#define ARM_DEBUG_SIZE		60


typedef struct {
    fp32 fdata[ARM_DEBUG_SIZE];
     char tail[4];
} Frame;


extern void debug_updata(void);

#endif
