#include "lan8720.h"
#include "delay.h"
#include "lwip_comm.h"
#include "pcf8574.h"
#include "usart.h"

//////////////////////////////////////////////////////////////////////////////////
// 鏈▼搴忓彧渚涘涔犱娇鐢紝鏈粡浣滆€呰鍙紝涓嶅緱鐢ㄤ簬鍏跺畠浠讳綍鐢ㄩ€�
// ALIENTEK STM32H7寮€鍙戞澘
// LAN8720椹卞姩浠ｇ爜
// 姝ｇ偣鍘熷瓙@ALIENTEK
// 鎶€鏈鍧�:www.openedv.com
// 鍒涘缓鏃ユ湡:2018/7/6
// 鐗堟湰锛歏1.0
// 鐗堟潈鎵€鏈夛紝鐩楃増蹇呯┒銆�
// Copyright(C) 骞垮窞甯傛槦缈肩數瀛愮鎶€鏈夐檺鍏徃 2014-2024
// All rights reserved
//////////////////////////////////////////////////////////////////////////////////

ETH_HandleTypeDef LAN8720_ETHHandle;

// 浠ュお缃戞弿杩扮鍜岀紦鍐插尯
__attribute__((at(0x30040000)))
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT]; // 浠ュお缃慠x DMA鎻忚堪绗�
__attribute__((at(0x30040060)))
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT]; // 浠ュお缃慣x DMA鎻忚堪绗�
__attribute__((at(0x30040200)))
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; // 浠ュお缃戞帴鏀剁紦鍐插尯

// 璁剧疆缃戠粶鎵€浣跨敤鐨�0X30040000鐨剅am鍐呭瓨淇濇姢
void NETMPU_Config(void) {
  MPU_Region_InitTypeDef MPU_InitStruct;

  HAL_MPU_Disable();
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x30040000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256B;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER5;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

// 鍒濆鍖朙AN8720
int32_t LAN8720_Init(void) {
  u8 macaddress[6];
  u32 timeout = 0;
  u32 regval = 0;
  u32 phylink = 0;
  int32_t status = LAN8720_STATUS_OK;

  // 纭欢澶嶄綅
  INTX_DISABLE(); // 鍏抽棴鎵€鏈変腑鏂紝澶嶄綅杩囩▼涓嶈兘琚墦鏂紒
  PCF8574_WriteBit(ETH_RESET_IO, 1); // 纭欢澶嶄綅
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_2, GPIO_PIN_RESET);

  delay_ms(100);
  PCF8574_WriteBit(ETH_RESET_IO, 0); // 澶嶄綅缁撴潫
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_2, GPIO_PIN_SET);

  delay_ms(100);
  INTX_ENABLE(); // 寮€鍚墍鏈変腑鏂�

  NETMPU_Config(); // MPU淇濇姢璁剧疆
  macaddress[0] = lwipdev.mac[0];
  macaddress[1] = lwipdev.mac[1];
  macaddress[2] = lwipdev.mac[2];
  macaddress[3] = lwipdev.mac[3];
  macaddress[4] = lwipdev.mac[4];
  macaddress[5] = lwipdev.mac[5];

  LAN8720_ETHHandle.Instance = ETH;                          // ETH
  LAN8720_ETHHandle.Init.MACAddr = macaddress;               // mac鍦板潃
  LAN8720_ETHHandle.Init.MediaInterface = HAL_ETH_RMII_MODE; // RMII鎺ュ彛
  LAN8720_ETHHandle.Init.RxDesc = DMARxDscrTab;              // 鍙戦€佹弿杩扮
  LAN8720_ETHHandle.Init.TxDesc = DMATxDscrTab;              // 鎺ユ敹鎻忚堪濡�
  LAN8720_ETHHandle.Init.RxBuffLen = ETH_MAX_PACKET_SIZE;    // 鎺ユ敹闀垮害
  HAL_ETH_Init(&LAN8720_ETHHandle);                          // 鍒濆鍖朎TH
  HAL_ETH_SetMDIOClockRange(&LAN8720_ETHHandle);

  if (LAN8720_WritePHY(LAN8720_BCR, LAN8720_BCR_SOFT_RESET) >=
      0) // LAN8720杞欢澶嶄綅
  {
    // 绛夊緟杞欢澶嶄綅瀹屾垚
    if (LAN8720_ReadPHY(LAN8720_BCR, &regval) >= 0) {
      while (regval & LAN8720_BCR_SOFT_RESET) {
        if (LAN8720_ReadPHY(LAN8720_BCR, &regval) < 0) {
          status = LAN8720_STATUS_READ_ERROR;
          break;
        }
        delay_ms(10);
        timeout++;
        if (timeout >= LAN8720_TIMEOUT)
          break; // 瓒呮椂璺冲嚭,5S
      }

    } else {
      status = LAN8720_STATUS_READ_ERROR;
    }
  } else {
    status = LAN8720_STATUS_WRITE_ERROR;
  }

  LAN8720_StartAutoNego(); // 寮€鍚嚜鍔ㄥ崗鍟嗗姛鑳�

  if (status == LAN8720_STATUS_OK) // 濡傛灉鍓嶉潰杩愯姝ｅ父灏卞欢鏃�1s
    delay_ms(1000);                // 绛夊緟1s

  // 绛夊緟缃戠粶杩炴帴鎴愬姛
  timeout = 0;
  while (LAN8720_GetLinkState() <= LAN8720_STATUS_LINK_DOWN) {
    delay_ms(10);
    timeout++;
    if (timeout >= LAN8720_TIMEOUT) {
      status = LAN8720_STATUS_LINK_DOWN;
      break; // 瓒呮椂璺冲嚭,5S
    }
  }
  phylink = LAN8720_GetLinkState();
  if (phylink == LAN8720_STATUS_100MBITS_FULLDUPLEX)
    printf("LAN8720:100Mb/s FullDuplex\r\n");
  else if (phylink == LAN8720_STATUS_100MBITS_HALFDUPLEX)
    printf("LAN8720:100Mb/s HalfDuplex\r\n");
  else if (phylink == LAN8720_STATUS_10MBITS_FULLDUPLEX)
    printf("LAN8720:10Mb/s FullDuplex\r\n");
  else if (phylink == LAN8720_STATUS_10MBITS_HALFDUPLEX)
    printf("LAN8720:10Mb/s HalfDuplex\r\n");
  return status;
}

extern void lwip_pkt_handle(void);

// 涓柇鏈嶅姟鍑芥暟
void ETH_IRQHandler(void) {
  lwip_pkt_handle();
  // 娓呴櫎涓柇鏍囧織浣�
  __HAL_ETH_DMA_CLEAR_IT(&LAN8720_ETHHandle,
                         ETH_DMA_NORMAL_IT); // 娓呴櫎DMA涓柇鏍囧織浣�
  __HAL_ETH_DMA_CLEAR_IT(&LAN8720_ETHHandle,
                         ETH_DMA_RX_IT); // 娓呴櫎DMA鎺ユ敹涓柇鏍囧織浣�
  __HAL_ETH_DMA_CLEAR_IT(&LAN8720_ETHHandle,
                         ETH_DMA_TX_IT); // 娓呴櫎DMA鎺ユ敹涓柇鏍囧織浣�
}

// ETH搴曞眰椹卞姩锛屽紩鑴氶厤缃紝鏃堕挓浣胯兘
// 姝ゅ嚱鏁颁細琚獺AL_ETH_Init()璋冪敤
// heth:ETH鍙ユ焺
void HAL_ETH_MspInit(ETH_HandleTypeDef *heth) {
  GPIO_InitTypeDef GPIO_Initure;

  __HAL_RCC_GPIOA_CLK_ENABLE();   // 寮€鍚疓PIOA鏃堕挓
  __HAL_RCC_GPIOB_CLK_ENABLE();   // 寮€鍚疓PIOB鏃堕挓
  __HAL_RCC_GPIOC_CLK_ENABLE();   // 寮€鍚疓PIOC鏃堕挓
  __HAL_RCC_GPIOG_CLK_ENABLE();   // 寮€鍚疓PIOG鏃堕挓
  __HAL_RCC_ETH1MAC_CLK_ENABLE(); // 浣胯兘ETH1 MAC鏃堕挓
  __HAL_RCC_ETH1TX_CLK_ENABLE();  // 浣胯兘ETH1鍙戦€佹椂閽�
  __HAL_RCC_ETH1RX_CLK_ENABLE();  // 浣胯兘ETH1鎺ユ敹鏃堕挓

  GPIO_Initure.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7;
  GPIO_Initure.Mode = GPIO_MODE_AF_PP;       // 鎺ㄦ尳澶嶇敤
  GPIO_Initure.Pull = GPIO_NOPULL;           // 涓嶅甫涓婁笅鎷�
  GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH; // 楂橀€�
  GPIO_Initure.Alternate = GPIO_AF11_ETH;    // 澶嶇敤涓篍TH鍔熻兘
  HAL_GPIO_Init(GPIOA, &GPIO_Initure);       // 鍒濆鍖�

  // PB11
  //    GPIO_Initure.Pin=GPIO_PIN_11;               //PB11
  //    HAL_GPIO_Init(GPIOB,&GPIO_Initure);         //濮嬪寲

  // PG11
  GPIO_Initure.Pin = GPIO_PIN_11;      // PG11
  HAL_GPIO_Init(GPIOG, &GPIO_Initure); // 濮嬪寲
  //

  // PC1,4,5
  GPIO_Initure.Pin = GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5; // PC1,4,5
  HAL_GPIO_Init(GPIOC, &GPIO_Initure);                     // 鍒濆鍖�

  // PG13,14
  GPIO_Initure.Pin = GPIO_PIN_13 | GPIO_PIN_14; // PG13,14
  HAL_GPIO_Init(GPIOG, &GPIO_Initure);          // 鍒濆鍖�

  HAL_NVIC_SetPriority(ETH_IRQn, 0, 0); // 缃戠粶涓柇浼樺厛绾у簲璇ラ珮涓€鐐�
  HAL_NVIC_EnableIRQ(ETH_IRQn);
}

// 璇诲彇PHY瀵勫瓨鍣ㄥ€�
// reg瑕佽鍙栫殑瀵勫瓨鍣ㄥ湴鍧€
// 杩斿洖鍊�:0 璇诲彇鎴愬姛锛�-1 璇诲彇澶辫触
int32_t LAN8720_ReadPHY(u16 reg, u32 *regval) {
  if (HAL_ETH_ReadPHYRegister(&LAN8720_ETHHandle, LAN8720_ADDR, reg, regval) !=
      HAL_OK)
    return -1;
  return 0;
}

// 鍚慙AN8720鎸囧畾瀵勫瓨鍣ㄥ啓鍏ュ€�
// reg:瑕佸啓鍏ョ殑瀵勫瓨鍣�
// value:瑕佸啓鍏ョ殑鍊�
// 杩斿洖鍊�:0 鍐欏叆姝ｅ父锛�-1 鍐欏叆澶辫触
int32_t LAN8720_WritePHY(u16 reg, u16 value) {
  u32 temp = value;
  if (HAL_ETH_WritePHYRegister(&LAN8720_ETHHandle, LAN8720_ADDR, reg, temp) !=
      HAL_OK)
    return -1;
  return 0;
}

// 鎵撳紑LAN8720 Power Down妯″紡
void LAN8720_EnablePowerDownMode(void) {
  u32 readval = 0;
  LAN8720_ReadPHY(LAN8720_BCR, &readval);
  readval |= LAN8720_BCR_POWER_DOWN;
  LAN8720_WritePHY(LAN8720_BCR, readval);
}

// 鍏抽棴LAN8720 Power Down妯″紡
void LAN8720_DisablePowerDownMode(void) {
  u32 readval = 0;
  LAN8720_ReadPHY(LAN8720_BCR, &readval);
  readval &= ~LAN8720_BCR_POWER_DOWN;
  LAN8720_WritePHY(LAN8720_BCR, readval);
}

// 寮€鍚疞AN8720鐨勮嚜鍗忓晢鍔熻兘
void LAN8720_StartAutoNego(void) {
  u32 readval = 0;
  LAN8720_ReadPHY(LAN8720_BCR, &readval);
  readval |= LAN8720_BCR_AUTONEGO_EN;
  LAN8720_WritePHY(LAN8720_BCR, readval);
}

// 浣胯兘鍥炴祴妯″紡
void LAN8720_EnableLoopbackMode(void) {
  u32 readval = 0;
  LAN8720_ReadPHY(LAN8720_BCR, &readval);
  readval |= LAN8720_BCR_LOOPBACK;
  LAN8720_WritePHY(LAN8720_BCR, readval);
}

// 鍏抽棴LAN8720鐨勫洖娴嬫ā寮�
void LAN8720_DisableLoopbackMode(void) {
  u32 readval = 0;
  LAN8720_ReadPHY(LAN8720_BCR, &readval);
  readval &= ~LAN8720_BCR_LOOPBACK;
  LAN8720_WritePHY(LAN8720_BCR, readval);
}

// 浣胯兘涓柇锛屼腑鏂簮鍙€�:LAN8720_ENERGYON_IT
//                      LAN8720_AUTONEGO_COMPLETE_IT
//                      LAN8720_REMOTE_FAULT_IT
//                      LAN8720_LINK_DOWN_IT
//                      LAN8720_AUTONEGO_LP_ACK_IT
//                      LAN8720_PARALLEL_DETECTION_FAULT_IT
//                      LAN8720_AUTONEGO_PAGE_RECEIVED_IT
void LAN8720_EnableIT(u32 interrupt) {
  u32 readval = 0;
  LAN8720_ReadPHY(LAN8720_IMR, &readval);
  readval |= interrupt;
  LAN8720_WritePHY(LAN8720_IMR, readval);
}

// 鍏抽棴涓柇锛屼腑鏂簮鍙€�:LAN8720_ENERGYON_IT
//                      LAN8720_AUTONEGO_COMPLETE_IT
//                      LAN8720_REMOTE_FAULT_IT
//                      LAN8720_LINK_DOWN_IT
//                      LAN8720_AUTONEGO_LP_ACK_IT
//                      LAN8720_PARALLEL_DETECTION_FAULT_IT
//                      LAN8720_AUTONEGO_PAGE_RECEIVED_IT
void LAN8720_DisableIT(u32 interrupt) {
  u32 readval = 0;
  LAN8720_ReadPHY(LAN8720_IMR, &readval);
  readval &= ~interrupt;
  LAN8720_WritePHY(LAN8720_IMR, readval);
}

// 娓呴櫎涓柇鏍囧織浣嶏紝璇诲瘎瀛樺櫒ISFR灏卞彲娓呴櫎涓柇鏍囧織浣�
void LAN8720_ClearIT(u32 interrupt) {
  u32 readval = 0;
  LAN8720_ReadPHY(LAN8720_ISFR, &readval);
}

// 鑾峰彇涓柇鏍囧織浣�
// 杩斿洖鍊硷紝1 涓柇鏍囧織浣嶇疆浣嶏紝
//         0 涓柇鏍囧織浣嶆竻闆�
u8 LAN8720_GetITStatus(u32 interrupt) {
  u32 readval = 0;
  u32 status = 0;
  LAN8720_ReadPHY(LAN8720_ISFR, &readval);
  if (readval & interrupt)
    status = 1;
  else
    status = 0;
  return status;
}

// 鑾峰彇LAN8720鐨勮繛鎺ョ姸鎬�
// 杩斿洖鍊硷細LAN8720_STATUS_LINK_DOWN              杩炴帴鏂紑
//         LAN8720_STATUS_AUTONEGO_NOTDONE       鑷姩鍗忓晢瀹屾垚
//         LAN8720_STATUS_100MBITS_FULLDUPLEX    100M鍏ㄥ弻宸�
//         LAN8720_STATUS_100MBITS_HALFDUPLEX    100M鍗婂弻宸�
//         LAN8720_STATUS_10MBITS_FULLDUPLEX     10M鍏ㄥ弻宸�
//         LAN8720_STATUS_10MBITS_HALFDUPLEX     10M鍗婂弻宸�
u32 readval = 0;

u32 LAN8720_GetLinkState(void) {

  // 璇诲彇涓ら亶锛岀‘淇濊鍙栨纭紒锛侊紒
  LAN8720_ReadPHY(LAN8720_BSR, &readval);
  LAN8720_ReadPHY(LAN8720_BSR, &readval);

  // 鑾峰彇杩炴帴鐘舵€�(纭欢锛岀綉绾跨殑杩炴帴锛屼笉鏄疶CP銆乁DP绛夎蒋浠惰繛鎺ワ紒)
  if ((readval & LAN8720_BSR_LINK_STATUS) == 0)
    return LAN8720_STATUS_LINK_DOWN;

  // 鑾峰彇鑷姩鍗忓晢鐘舵€�
  LAN8720_ReadPHY(LAN8720_BCR, &readval);
  if ((readval & LAN8720_BCR_AUTONEGO_EN) !=
      LAN8720_BCR_AUTONEGO_EN) // 鏈娇鑳借嚜鍔ㄥ崗鍟�
  {
    if (((readval & LAN8720_BCR_SPEED_SELECT) == LAN8720_BCR_SPEED_SELECT) &&
        ((readval & LAN8720_BCR_DUPLEX_MODE) == LAN8720_BCR_DUPLEX_MODE))
      return LAN8720_STATUS_100MBITS_FULLDUPLEX;
    else if ((readval & LAN8720_BCR_SPEED_SELECT) == LAN8720_BCR_SPEED_SELECT)
      return LAN8720_STATUS_100MBITS_HALFDUPLEX;
    else if ((readval & LAN8720_BCR_DUPLEX_MODE) == LAN8720_BCR_DUPLEX_MODE)
      return LAN8720_STATUS_10MBITS_FULLDUPLEX;
    else
      return LAN8720_STATUS_10MBITS_HALFDUPLEX;
  } else // 浣胯兘浜嗚嚜鍔ㄥ崗鍟�
  {
    LAN8720_ReadPHY(LAN8720_PHYSCSR, &readval);
    if ((readval & LAN8720_PHYSCSR_AUTONEGO_DONE) == 0)
      return LAN8720_STATUS_AUTONEGO_NOTDONE;
    if ((readval & LAN8720_PHYSCSR_HCDSPEEDMASK) == LAN8720_PHYSCSR_100BTX_FD)
      return LAN8720_STATUS_100MBITS_FULLDUPLEX;
    else if ((readval & LAN8720_PHYSCSR_HCDSPEEDMASK) ==
             LAN8720_PHYSCSR_100BTX_HD)
      return LAN8720_STATUS_100MBITS_HALFDUPLEX;
    else if ((readval & LAN8720_PHYSCSR_HCDSPEEDMASK) ==
             LAN8720_PHYSCSR_10BT_FD)
      return LAN8720_STATUS_10MBITS_FULLDUPLEX;
    else
      return LAN8720_STATUS_10MBITS_HALFDUPLEX;
  }
}

// 璁剧疆LAN8720鐨勮繛鎺ョ姸鎬�
// 鍙傛暟linkstate锛歀AN8720_STATUS_100MBITS_FULLDUPLEX 100M鍏ㄥ弻宸�
//                LAN8720_STATUS_100MBITS_HALFDUPLEX 100M鍗婂弻宸�
//                LAN8720_STATUS_10MBITS_FULLDUPLEX  10M鍏ㄥ弻宸�
//                LAN8720_STATUS_10MBITS_HALFDUPLEX  10M鍗婂弻宸�
void LAN8720_SetLinkState(u32 linkstate) {

  u32 bcrvalue = 0;
  LAN8720_ReadPHY(LAN8720_BCR, &bcrvalue);
  // 鍏抽棴杩炴帴閰嶇疆锛屾瘮濡傝嚜鍔ㄥ崗鍟嗭紝閫熷害鍜屽弻宸�
  bcrvalue &= ~(LAN8720_BCR_AUTONEGO_EN | LAN8720_BCR_SPEED_SELECT |
                LAN8720_BCR_DUPLEX_MODE);
  if (linkstate == LAN8720_STATUS_100MBITS_FULLDUPLEX) // 100M鍏ㄥ弻宸�
    bcrvalue |= (LAN8720_BCR_SPEED_SELECT | LAN8720_BCR_DUPLEX_MODE);
  else if (linkstate == LAN8720_STATUS_100MBITS_HALFDUPLEX) // 100M鍗婂弻宸�
    bcrvalue |= LAN8720_BCR_SPEED_SELECT;
  else if (linkstate == LAN8720_STATUS_10MBITS_FULLDUPLEX) // 10M鍏ㄥ弻宸�
    bcrvalue |= LAN8720_BCR_DUPLEX_MODE;

  LAN8720_WritePHY(LAN8720_BCR, bcrvalue);
}
