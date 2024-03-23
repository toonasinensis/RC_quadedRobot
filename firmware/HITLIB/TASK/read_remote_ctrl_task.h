#ifndef __READ_REMOTE_CTRL_TASK_H__
#define __READ_REMOTE_CTRL_TASK_H__

#include "hit_global_type.h"
#include "remote_control.h"

/*手柄 间断按键 宏定义*/
#define PRESS_LEFT (g_stJsValue.aucKeyPress[15] == 1)
#define PRESS_RIGHT (g_stJsValue.aucKeyPress[13] == 1)
#define PRESS_UP (g_stJsValue.aucKeyPress[12] == 1)
#define PRESS_DOWN (g_stJsValue.aucKeyPress[14] == 1)
#define PRESS_B1 (g_stJsValue.aucKeyPress[4] == 1)
#define PRESS_B2 (g_stJsValue.aucKeyPress[5] == 1)
#define PRESS_B3 (g_stJsValue.aucKeyPress[6] == 1)
#define PRESS_B4 (g_stJsValue.aucKeyPress[7] == 1)
#define PRESS_L1 (g_stJsValue.aucKeyPress[2] == 1)
#define PRESS_L2 (g_stJsValue.aucKeyPress[0] == 1)
#define PRESS_R1 (g_stJsValue.aucKeyPress[3] == 1)
#define PRESS_R2 (g_stJsValue.aucKeyPress[1] == 1)
#define PRESS_START (g_stJsValue.aucKeyPress[11] == 1)
#define PRESS_SELECT (g_stJsValue.aucKeyPress[8] == 1)


/*键盘 宏定义*/
//#define PRESS_KEY_NUMLOCK (g_usKeyValue == 0xFF77)
//#define PRESS_KEY_0 (g_usKeyValue == 0xFF70)
//#define PRESS_KEY_1 (g_usKeyValue == 0xFF69)
//#define PRESS_KEY_2 (g_usKeyValue == 0xFF72)
//#define PRESS_KEY_3 (g_usKeyValue == 0xFF7A)
//#define PRESS_KEY_4 (g_usKeyValue == 0xFF6B)
//#define PRESS_KEY_5 (g_usKeyValue == 0xFF73)
//#define PRESS_KEY_6 (g_usKeyValue == 0xFF74)
//#define PRESS_KEY_7 (g_usKeyValue == 0xFF6C)
//#define PRESS_KEY_8 (g_usKeyValue == 0xFF75)
//#define PRESS_KEY_9 (g_usKeyValue == 0xFF7D)
//#define PRESS_KEY_BS (g_usKeyValue == 0xFF66)
//#define PRESS_KEY_ENTER (g_usKeyValue == 0xFF5A)
//#define PRESS_KEY_INC (g_usKeyValue == 0xFF79)
//#define PRESS_KEY_DEC (g_usKeyValue == 0xFF7B)
//#define PRESS_KEY_MUL (g_usKeyValue == 0xFF7C)
//#define PRESS_KEY_DOT (g_usKeyValue == 0xFF71)
//#define PRESS_KEY_DIV (g_usKeyValue == 0xFF4A)
//#define PRESS_KEY_NULL (g_usKeyValue == 0xFFFF)

/*键盘 宏定义  2024 RC */
//    键盘值  20 19 23 22 21
//           0x14 0x0E 0x08    0x01 0x1F 0x19
//           0x18 0x12 0x0C    0x05 0x23 0x1D
//           0x17 0x11 0x0B    0x04 0x22 0x1C
// 0x16 0x0F 0x10 0x09 0x0A    0x03 0x02 0x21 0x20 0x1B
// 0x15 0x13 0x0D 0x07 0x25    0x26 0x06 0x24 0x1E 0x1A
#define PRESS_KEY_11 (g_usKeyValue == 0xFF14)
#define PRESS_KEY_12 (g_usKeyValue == 0xFF0E)
#define PRESS_KEY_13 (g_usKeyValue == 0xFF08)
#define PRESS_KEY_14 (g_usKeyValue == 0xFF01)
#define PRESS_KEY_15 (g_usKeyValue == 0xFF1F)
#define PRESS_KEY_16 (g_usKeyValue == 0xFF19)

#define PRESS_KEY_21 (g_usKeyValue == 0xFF18)
#define PRESS_KEY_22 (g_usKeyValue == 0xFF12)
#define PRESS_KEY_23 (g_usKeyValue == 0xFF0C)
#define PRESS_KEY_24 (g_usKeyValue == 0xFF05)
#define PRESS_KEY_25 (g_usKeyValue == 0xFF23)
#define PRESS_KEY_26 (g_usKeyValue == 0xFF1D)

#define PRESS_KEY_31 (g_usKeyValue == 0xFF17)
#define PRESS_KEY_32 (g_usKeyValue == 0xFF11)
#define PRESS_KEY_33 (g_usKeyValue == 0xFF0B)
#define PRESS_KEY_34 (g_usKeyValue == 0xFF04)
#define PRESS_KEY_35 (g_usKeyValue == 0xFF22)
#define PRESS_KEY_36 (g_usKeyValue == 0xFF1C)

#define PRESS_KEY_40 (g_usKeyValue == 0xFF16)
#define PRESS_KEY_41 (g_usKeyValue == 0xFF0F)
#define PRESS_KEY_42 (g_usKeyValue == 0xFF10)
#define PRESS_KEY_43 (g_usKeyValue == 0xFF09)
#define PRESS_KEY_44 (g_usKeyValue == 0xFF0A)
#define PRESS_KEY_45 (g_usKeyValue == 0xFF03)
#define PRESS_KEY_46 (g_usKeyValue == 0xFF02)
#define PRESS_KEY_47 (g_usKeyValue == 0xFF21)
#define PRESS_KEY_48 (g_usKeyValue == 0xFF20)
#define PRESS_KEY_49 (g_usKeyValue == 0xFF1B)

#define PRESS_KEY_50 (g_usKeyValue == 0xFF15)
#define PRESS_KEY_51 (g_usKeyValue == 0xFF13)
#define PRESS_KEY_52 (g_usKeyValue == 0xFF0D)
#define PRESS_KEY_53 (g_usKeyValue == 0xFF07)
#define PRESS_KEY_54 (g_usKeyValue == 0xFF25)
#define PRESS_KEY_55 (g_usKeyValue == 0xFF26)
#define PRESS_KEY_56 (g_usKeyValue == 0xFF06)
#define PRESS_KEY_57 (g_usKeyValue == 0xFF24)
#define PRESS_KEY_58 (g_usKeyValue == 0xFF1E)
#define PRESS_KEY_59 (g_usKeyValue == 0xFF1A)

typedef enum {
    DEBUG,
    CALIBRATION,
    PATH,
    ACTION,
    PATHPLANNING,
} keyboard_mode_e;



typedef enum {
    FETCH_POINT,
    HIT_RT1,
    HIT_RT2,
    HIT_START3,
    HIT_LT4,
} waypoint_e;

extern uint16_t g_usKeyValue;
extern uint16_t g_usstartKeyValue;
extern int16_t M_SPEED;  //手操速度


extern void Js_Deal(void);   //手柄处理函数
extern void Key_Deal(void);  //键盘处理函数
extern void Auto_Key_Deal(void);

extern bool flag_auto_fetch_all;

extern keyboard_mode_e keyboard_mode;

extern waypoint_e waypoint;


#endif
