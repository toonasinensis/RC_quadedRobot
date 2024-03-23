#include "remote_control.h"

//手柄键值和摇杆结构体
ST_JS_VALUE g_stJsValue = {0xffff,{0},{0},{0},{0},0,0x8080,0x8080};

//wlan手柄键值和摇杆结构体
ST_JS_VALUE Wlan_g_stJsValue = {0,{0},{0},{0},{0},0,0x8080,0x8080};

/******************************************************************
无线手柄相关函数
**********************************************************************/
uint8_t tmp_buf[33];
uint8_t tmp_buf_11;

/************************************************************
函数名 :CalSpeedByJoyStick()
函数功能:通过手柄摇杆值分配速度
输入: pstJsRockerValue 指向手柄的结构体
      pstVelt 指向速度结构体的指针
      ucGateX 手柄的X方向阈值,只有超过这个阈值才运动
      ucGateY 手柄Y方向阈值
	  ucGateW 手柄W方向(旋转)阈值
      ssXSpedLimit X方向上速度上限
      ssYSpedLimit Y方向上速度上限
	  ssWSpedLimit W方向上速度上限
输出：无
备注：这里要考虑机器人的走向和实际操作者的位置,遥感的输入为0~132*2
************************************************************/
int flag_coordinate=0;
int max = 0;
int min = 0;
int wyy;
void CalSpeedByJoyStick(ST_JS_VALUE *g_stJsValue, cNav *p_nav, uint8_t ucGateX,
								int16_t ssXSpedLimit,uint8_t ucGateY,int16_t ssYSpedLimit,
													uint8_t ucGateW,int16_t ssWSpedLimit)
{
    uint8_t ucJSTmp;
    int16_t Vx,Vy,Vw;//Vt;
	const fp32 smooth = 30;					//平滑等级，一阶低通滤波器截止频率

	static ST_LPF FJx = {0, 0, 0, smooth, 0.002f};//需要pre_out，故需要设为静态变量
	static ST_LPF FJy = {0, 0, 0, smooth, 0.002f};
	static ST_LPF FJw = {0, 0, 0, smooth, 0.002f};
	
    //ucJSTmp = g_stJsValue->usJsLeft>>8;//读取手柄左摇杆的高八位，对应左右
	ucJSTmp = g_stJsValue->usJsLeft>>8;
	wyy = ucJSTmp;
	if(max < ucJSTmp)
	{
		max = ucJSTmp;
	}		
	else if(min > ucJSTmp)
	{
		min = ucJSTmp;
	}
    //Vx=(ucJSTmp - JS_MID_POS);
	Vx = (ucJSTmp - JS_MID_POS);

    if(fabs((fp32)Vx) < ucGateX)
    {
        FJx.in = 0;
    }
    else if(Vx>=ucGateX)
    {
        FJx.in = (Vx-ucGateX) * ssXSpedLimit / (JS_MID_POS - ucGateX);
    }
    else if(Vx<=-ucGateX)
    {
        FJx.in = (Vx+ucGateX) * ssXSpedLimit / (JS_MID_POS - ucGateX);
    }

    //ucJSTmp = g_stJsValue->usJsLeft & (0x00ff);//读取左摇杆的低八位，对应前后
 	ucJSTmp = g_stJsValue->usJsLeft & (0x00ff);
   Vy = (ucJSTmp-JS_MID_POS);

    if(fabs((fp32)Vy) < ucGateY)
    {
        FJy.in = 0;
    }
    else if(Vy >= ucGateY)
    {
        FJy.in = (Vy-ucGateY) * ssYSpedLimit / (JS_MID_POS - ucGateY);
    }
    else if( Vy <= -ucGateY)
    {
        FJy.in = (Vy+ucGateY) * ssYSpedLimit / (JS_MID_POS - ucGateY);
    }
//    ucJSTmp = g_stJsValue->usJsRight>>8;//读取右摇杆的高八位，对应旋转
    ucJSTmp = g_stJsValue->usJsRight & (0x00ff);//读取右摇杆的低八位，对应旋转
    Vw = (ucJSTmp-0x7B);//JS_MID_POS);

    if(fabs((fp32)Vw) < ucGateW)
    {
        FJw.in = 0;
    }
    else if(Vw >= ucGateW)
    {
        FJw.in = (Vw-ucGateW) * ssWSpedLimit / (0x7B - ucGateW);
    }
    else if(Vw<=-ucGateW)
    {
        FJw.in = (Vw+ucGateW) * ssWSpedLimit / (0x7B - ucGateW);
    }
	
	LpFilter(&FJx);
	LpFilter(&FJy);
	LpFilter(&FJw);
	
	
	if(flag_coordinate==1)
	{
		p_nav->expect_robot_global_velt.fpVx = FJy.out;
	  p_nav->expect_robot_global_velt.fpVy = -FJx.out;
	  p_nav->expect_robot_global_velt.fpW = FJw.out / 7.0f;//本来是除以10
	}	
	else if(flag_coordinate==3)
	{
		p_nav->expect_robot_global_velt.fpVx = -FJx.out;
	  p_nav->expect_robot_global_velt.fpVy = -FJy.out;
	  p_nav->expect_robot_global_velt.fpW = FJw.out / 7.0f;//本来是除以10
	}
	else if(flag_coordinate==2)
	{
		p_nav->expect_robot_global_velt.fpVx = FJx.out;
	  p_nav->expect_robot_global_velt.fpVy = FJy.out;
	  p_nav->expect_robot_global_velt.fpW = FJw.out / 7.0f;//本来是除以10
	}
	else
	{
		p_nav->expect_robot_global_velt.fpVx = -FJy.out;
	  p_nav->expect_robot_global_velt.fpVy = FJx.out;
	  p_nav->expect_robot_global_velt.fpW = FJw.out / 7.0f;//本来是除以10
	}
	
}

/******************************************************************
函数名: ReadWlanJsValue(ST_JS_VALUE *pstJsValue)
函数功能: 手柄键值处理函数
输入:  手柄键值结构体
输出:
备注:  500内处理为单击，超过后处理为连击，且每100ms使按键有效一次
******************************************************************/
void ReadWlanJsValue(ST_JS_VALUE *pstJsValue)
{

	if(remote_rx_flag)//接收到数
	{
	  memcpy(tmp_buf, usart4_efr.num, REMOTE_RX_DATA_LEN);
		
		if(((tmp_buf[0] == 0x73) || (tmp_buf[0] == 0x41)) && (tmp_buf[1] == 0x5A))
		{
			uint8_t i = 0;
			pstJsValue->usJsState = Wlan_JOYSTICK_STATE;
			pstJsValue->usJsKey = Wlan_JOYSTICK_RESERVED;
		
			for(i = 0; i < 16; ++i)
			{
				if((pstJsValue->usJsKey & (1 << i)) == 0)
				{
					if(pstJsValue->auiPressDuration[i] == 0)    //按下后，第一次读取
					{
						pstJsValue->auiPressDuration[i] = 5;
						pstJsValue->uiStartTime[i] = TIM7->CNT;//T1TC;
						pstJsValue->aucKeyPress[i] = 1;
					}
					else
					{
						pstJsValue->uiCurTime[i] = TIM7->CNT;//T1TC;
						if(pstJsValue->uiCurTime[i] - pstJsValue->uiStartTime[i]
								> pstJsValue->auiPressDuration[i] * 100000)    //第一次为500ms,之后100ms一次
						{
							pstJsValue->auiPressDuration[i] ++;
							pstJsValue->aucKeyPress[i] = 1;
						}
						else
						{
							pstJsValue->aucKeyPress[i] = 0;
						}
					}/*检测按下的时间，判断是否处理为连击*/
				}
				else
				{
					pstJsValue->auiPressDuration[i] = 0;
					pstJsValue->uiStartTime[i] = 0;
					pstJsValue->uiCurTime[i] = 0;
					pstJsValue->aucKeyPress[i] = 0;
				}/*检测是否被按下*/
			}

				pstJsValue->usJsLeft = Wlan_JOYSTICK_LEFT;
				pstJsValue->usJsRight = Wlan_JOYSTICK_RIGTH;			

//			if(pstJsValue->usJsState == 0x73)          //红灯模式
//			{
//				pstJsValue->usJsLeft = Wlan_JOYSTICK_LEFT;
//				pstJsValue->usJsRight = Wlan_JOYSTICK_RIGTH;
//			}
//			else if(pstJsValue->usJsState == 0x41)
//			{
//				pstJsValue->usJsLeft = 0x8080;
//				pstJsValue->usJsRight = 0x8080;
//			}
		}
		
	}
	else 
	{
		pstJsValue->usJsKey = 0xFFFF;
		pstJsValue->usJsLeft = 0x8080; 			
		pstJsValue->usJsRight = 0x8080; 		
		pstJsValue->usJsState =  0x00;
	}
	
	if((tmp_buf[9] == 0xAA) && (tmp_buf[10] == 0x88))
	{
		tmp_buf_11 = tmp_buf[11];
	}
}

    static uint8_t Wlan_ucKeyStatic = 0;
    static uint32_t Wlan_uiStartTime = 0;
    static uint16_t Wlan_usKeyPre = 0xFFFF;
    uint32_t uiCurTime = 0;
    uint16_t usKey = 0;
/******************************************************************
函数名: ReadWlanKeyValue(uint16_t *Wlan_pusKeyValue)
函数功能: WLAN键盘键值处理函数
输入: WALN 键盘键值结构体
输出:
备注:
******************************************************************/
void ReadWlanKeyValue(uint16_t *Wlan_pusKeyValue)
{


	usKey = Wlan_PSKEY;
    if(Wlan_ucKeyStatic == 0)//自由状态
    {
        if(usKey != 0xFFFF && usKey != Wlan_usKeyPre)
        {
            *Wlan_pusKeyValue = usKey;
//            Wlan_uiStartTime = TIM7->CNT;
            Wlan_ucKeyStatic = 1;//有数状态
        }
        Wlan_usKeyPre = usKey;
    }
    else if(Wlan_ucKeyStatic == 1)
    {
        *Wlan_pusKeyValue = 0xFFFF;
        Wlan_ucKeyStatic = 2;//等待状态
    }
    else if(Wlan_ucKeyStatic == 2)
    {
//        uiCurTime = TIM7->CNT;
//        if(uiCurTime - Wlan_uiStartTime > 20000)
//        {
            Wlan_ucKeyStatic = 0;
//        }
    }
}
