#include "remote_control.h"

//�ֱ���ֵ��ҡ�˽ṹ��
ST_JS_VALUE g_stJsValue = {0xffff,{0},{0},{0},{0},0,0x8080,0x8080};

//wlan�ֱ���ֵ��ҡ�˽ṹ��
ST_JS_VALUE Wlan_g_stJsValue = {0,{0},{0},{0},{0},0,0x8080,0x8080};

/******************************************************************
�����ֱ���غ���
**********************************************************************/
uint8_t tmp_buf[33];
uint8_t tmp_buf_11;

/************************************************************
������ :CalSpeedByJoyStick()
��������:ͨ���ֱ�ҡ��ֵ�����ٶ�
����: pstJsRockerValue ָ���ֱ��Ľṹ��
      pstVelt ָ���ٶȽṹ���ָ��
      ucGateX �ֱ���X������ֵ,ֻ�г��������ֵ���˶�
      ucGateY �ֱ�Y������ֵ
	  ucGateW �ֱ�W����(��ת)��ֵ
      ssXSpedLimit X�������ٶ�����
      ssYSpedLimit Y�������ٶ�����
	  ssWSpedLimit W�������ٶ�����
�������
��ע������Ҫ���ǻ����˵������ʵ�ʲ����ߵ�λ��,ң�е�����Ϊ0~132*2
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
	const fp32 smooth = 30;					//ƽ���ȼ���һ�׵�ͨ�˲�����ֹƵ��

	static ST_LPF FJx = {0, 0, 0, smooth, 0.002f};//��Ҫpre_out������Ҫ��Ϊ��̬����
	static ST_LPF FJy = {0, 0, 0, smooth, 0.002f};
	static ST_LPF FJw = {0, 0, 0, smooth, 0.002f};
	
    //ucJSTmp = g_stJsValue->usJsLeft>>8;//��ȡ�ֱ���ҡ�˵ĸ߰�λ����Ӧ����
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

    //ucJSTmp = g_stJsValue->usJsLeft & (0x00ff);//��ȡ��ҡ�˵ĵͰ�λ����Ӧǰ��
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
//    ucJSTmp = g_stJsValue->usJsRight>>8;//��ȡ��ҡ�˵ĸ߰�λ����Ӧ��ת
    ucJSTmp = g_stJsValue->usJsRight & (0x00ff);//��ȡ��ҡ�˵ĵͰ�λ����Ӧ��ת
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
	  p_nav->expect_robot_global_velt.fpW = FJw.out / 7.0f;//�����ǳ���10
	}	
	else if(flag_coordinate==3)
	{
		p_nav->expect_robot_global_velt.fpVx = -FJx.out;
	  p_nav->expect_robot_global_velt.fpVy = -FJy.out;
	  p_nav->expect_robot_global_velt.fpW = FJw.out / 7.0f;//�����ǳ���10
	}
	else if(flag_coordinate==2)
	{
		p_nav->expect_robot_global_velt.fpVx = FJx.out;
	  p_nav->expect_robot_global_velt.fpVy = FJy.out;
	  p_nav->expect_robot_global_velt.fpW = FJw.out / 7.0f;//�����ǳ���10
	}
	else
	{
		p_nav->expect_robot_global_velt.fpVx = -FJy.out;
	  p_nav->expect_robot_global_velt.fpVy = FJx.out;
	  p_nav->expect_robot_global_velt.fpW = FJw.out / 7.0f;//�����ǳ���10
	}
	
}

/******************************************************************
������: ReadWlanJsValue(ST_JS_VALUE *pstJsValue)
��������: �ֱ���ֵ������
����:  �ֱ���ֵ�ṹ��
���:
��ע:  500�ڴ���Ϊ��������������Ϊ��������ÿ100msʹ������Чһ��
******************************************************************/
void ReadWlanJsValue(ST_JS_VALUE *pstJsValue)
{

	if(remote_rx_flag)//���յ���
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
					if(pstJsValue->auiPressDuration[i] == 0)    //���º󣬵�һ�ζ�ȡ
					{
						pstJsValue->auiPressDuration[i] = 5;
						pstJsValue->uiStartTime[i] = TIM7->CNT;//T1TC;
						pstJsValue->aucKeyPress[i] = 1;
					}
					else
					{
						pstJsValue->uiCurTime[i] = TIM7->CNT;//T1TC;
						if(pstJsValue->uiCurTime[i] - pstJsValue->uiStartTime[i]
								> pstJsValue->auiPressDuration[i] * 100000)    //��һ��Ϊ500ms,֮��100msһ��
						{
							pstJsValue->auiPressDuration[i] ++;
							pstJsValue->aucKeyPress[i] = 1;
						}
						else
						{
							pstJsValue->aucKeyPress[i] = 0;
						}
					}/*��ⰴ�µ�ʱ�䣬�ж��Ƿ���Ϊ����*/
				}
				else
				{
					pstJsValue->auiPressDuration[i] = 0;
					pstJsValue->uiStartTime[i] = 0;
					pstJsValue->uiCurTime[i] = 0;
					pstJsValue->aucKeyPress[i] = 0;
				}/*����Ƿ񱻰���*/
			}

				pstJsValue->usJsLeft = Wlan_JOYSTICK_LEFT;
				pstJsValue->usJsRight = Wlan_JOYSTICK_RIGTH;			

//			if(pstJsValue->usJsState == 0x73)          //���ģʽ
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
������: ReadWlanKeyValue(uint16_t *Wlan_pusKeyValue)
��������: WLAN���̼�ֵ������
����: WALN ���̼�ֵ�ṹ��
���:
��ע:
******************************************************************/
void ReadWlanKeyValue(uint16_t *Wlan_pusKeyValue)
{


	usKey = Wlan_PSKEY;
    if(Wlan_ucKeyStatic == 0)//����״̬
    {
        if(usKey != 0xFFFF && usKey != Wlan_usKeyPre)
        {
            *Wlan_pusKeyValue = usKey;
//            Wlan_uiStartTime = TIM7->CNT;
            Wlan_ucKeyStatic = 1;//����״̬
        }
        Wlan_usKeyPre = usKey;
    }
    else if(Wlan_ucKeyStatic == 1)
    {
        *Wlan_pusKeyValue = 0xFFFF;
        Wlan_ucKeyStatic = 2;//�ȴ�״̬
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
