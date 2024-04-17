#ifndef __GIMBAL_APP_H__
#define __GIMBAL_APP_H__

#include "main.h"

#define GM_PITCH_MIDPOS   2710
#define GM_PITCH_WIDE1    50
#define GM_PITCH_WIDE2    700

#define GM_YAW_MIDPOS     0
#define GM_YAW_WIDE       3800


#define GM_PITCH_MAXPOS   GM_PITCH_MIDPOS + GM_PITCH_WIDE1
#define GM_PITCH_MINPOS   GM_PITCH_MIDPOS - GM_PITCH_WIDE2

#define Shoot_Step 42130

void GimbalControl(void);





#endif




