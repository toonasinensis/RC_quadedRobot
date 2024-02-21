#include "udp_demo.h"
#include "delay.h"
#include "key.h"
#include "lcd.h"
#include "led.h"
#include "malloc.h"
#include "stdio.h"
#include "string.h"
#include "udp_comm.h"
#include "usart.h"

// UDP鎺ユ敹鏁版嵁缂撳啿鍖�
u8 udp_demo_recvbuf[UDP_DEMO_RX_BUFSIZE]; // UDP鎺ユ敹鏁版嵁缂撳啿鍖�
// UDP鍙戦€佹暟鎹唴瀹�
u8 udp_send_buffer_quad[200];
const u8 *tcp_demo_sendbuf = "Apollo STM32F4/F7/H7 UDP demo send data\r\n";

// UDP 娴嬭瘯鍏ㄥ眬鐘舵€佹爣璁板彉閲�
// bit7:娌℃湁鐢ㄥ埌
// bit6:0,娌℃湁鏀跺埌鏁版嵁;1,鏀跺埌鏁版嵁浜�.
// bit5:0,娌℃湁杩炴帴涓�;1,杩炴帴涓婁簡.
// bit4~0:淇濈暀
u8 udp_demo_flag;

// 璁剧疆杩滅IP鍦板潃
extern uint8_t fast_send;
void udp_demo_set_remoteip(void) {
  // u8 *tbuf;
  // u16 xoff;
  // u8 key;

  // tbuf = mymalloc(SRAMIN, 400); // 鐢宠鍐呭瓨
  // if (tbuf == NULL)
  //   return;
  // 鍓嶄笁涓狪P淇濇寔鍜孌HCP寰楀埌鐨処P涓€鑷�
  lwipdev.remoteip[0] = lwipdev.ip[0];
  lwipdev.remoteip[1] = lwipdev.ip[1];
  lwipdev.remoteip[2] = lwipdev.ip[2];

  // myfree(SRAMIN, tbuf);
}

// UDP娴嬭瘯
u8 udp_send_flag;

void udp_demo_test(void) {
  err_t err;
  struct udp_pcb *udppcb; // 瀹氫箟涓€涓猅CP鏈嶅姟鍣ㄦ帶鍒跺潡
  // struct ip_addr rmtipaddr; // 杩滅ip鍦板潃

  u8 *tbuf;
  u8 key;
  u8 res = 0;
  u16 t = 0;

  udp_demo_set_remoteip(); // 鍏堥€夋嫨IP

  tbuf = mymalloc(SRAMIN, 400); // 鐢宠鍐呭瓨
  if (tbuf == NULL)
    return; // 鍐呭瓨鐢宠澶辫触浜�,鐩存帴閫€鍑�

  udppcb = udp_new();
  if (udppcb) // 鍒涘缓鎴愬姛
  {
    // IP4_ADDR(&rmtipaddr, lwipdev.remoteip[0], lwipdev.remoteip[1],
    //          lwipdev.remoteip[2], lwipdev.remoteip[3]);
    // NOTE: No need to connect right now
    // UDP瀹㈡埛绔繛鎺ュ埌鎸囧畾IP鍦板潃鍜岀鍙ｅ彿鐨勬湇鍔″櫒
    // err = udp_connect(udppcb, &rmtipaddr, UDP_REMOTE_PORT);
    err = udp_bind(udppcb, IP_ADDR_ANY, UDP_LOCAL_PORT);
    if (err == ERR_OK) {
      udp_recv(udppcb, udp_demo_recv, NULL); // 娉ㄥ唽鎺ユ敹鍥炶皟鍑芥暟
      // 鏍囪杩炴帴涓婁簡(UDP鏄潪鍙潬杩炴帴,杩欓噷浠呬粎琛ㄧず鏈湴UDP宸茬粡鍑嗗濂�)
      udp_demo_flag |= 1 << 5; // 鏍囪宸茬粡杩炴帴涓�
    } else
      res = 1;
  } else
    res = 1;

  // Make sure udp is disconnected
  udp_disconnect(udppcb);

  fast_send = 1;
  while (1) {
    // key = KEY_Scan(0);
    // if (key == WKUP_PRES || udp_send_flag == 1) // KEY0鎸変笅浜�,鍙戦€佹暟鎹�
    // {
    //   udp_send_flag = 0;
    //   udp_demo_senddata(udppcb);
    // }
    // if (udp_demo_flag & 1 << 6) // 鏄惁鏀跺埌鏁版嵁?
    // {
    //   udp_demo_flag &= ~(1 << 6); // 鏍囪鏁版嵁宸茬粡琚鐞嗕簡.
    // }
    lwip_periodic_handle();
    delay_us(100);
    // t++;
    // if (t >= 1000) {
    //   t = 0;
    //   LED0_Toggle;
    // }
  }
  udp_demo_connection_close(udppcb);
  myfree(SRAMIN, tbuf);
}
extern uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; // 浠ュお缃戞帴鏀剁紦鍐插尯

int udp_cnt=0;

// UDP鍥炶皟鍑芥暟
void udp_demo_recv(void *arg, struct udp_pcb *upcb, struct pbuf *p,
                   struct ip_addr *addr, u16_t port) {
	udp_cnt++;
  SCB_InvalidateDCache();
  u32 data_len = 0;
  u8 success_rev_flag = 0;
  struct pbuf *q;

  // get motor command from PC
  if (p != NULL) // 鎺ユ敹鍒颁笉涓虹┖鐨勬暟鎹椂
  {
			SCB_InvalidateDCache_by_Addr((uint32_t *)Rx_Buff, ETH_RX_DESC_CNT*ETH_MAX_PACKET_SIZE);

    memset(udp_demo_recvbuf, 0, UDP_DEMO_RX_BUFSIZE); // 鏁版嵁鎺ユ敹缂撳啿鍖烘竻闆�
    for (q = p; q != NULL; q = q->next) {
      if (q->len > (UDP_DEMO_RX_BUFSIZE - data_len)) {
        memcpy(udp_demo_recvbuf + data_len, q->payload,
               (UDP_DEMO_RX_BUFSIZE - data_len)); // 鎷疯礉鏁版嵁
      } else {
        memcpy(udp_demo_recvbuf + data_len, q->payload, q->len);
      }
      data_len += q->len;
      if (data_len > UDP_DEMO_RX_BUFSIZE)
        break; // 瓒呭嚭TCP瀹㈡埛绔帴鏀舵暟缁�,璺冲嚭
    }
    // Set Motor Cmd
    memcpy(&udp_receive_data, udp_demo_recvbuf, sizeof(udp_receive_data));
    if (crc32_core((uint8_t *)&udp_receive_data,
                   sizeof(udp_receive_data) / 4 - 1) ==
        udp_receive_data.check_digit) {
      LED0_Toggle;
			
					
      for (int i = 0; i < 6; ++i) {
						if(udp_receive_data.state==1)//normal
					{
						leg[i].hip_motor.command.mode =10;
						leg[i].thigh_motor.command.mode =10;
						leg[i].knee_motor.command.mode =10;
					}
					else
					{
						leg[i].hip_motor.command.mode =0;
						leg[i].thigh_motor.command.mode =0;
						leg[i].knee_motor.command.mode =0;
					}
        udp_motor_type2raw_motor_type(&udp_receive_data.udp_motor_send[i * 3],
                                      &leg[i].hip_motor.command);
        udp_motor_type2raw_motor_type(
            &udp_receive_data.udp_motor_send[i * 3 + 1],
            &leg[i].thigh_motor.command);
        udp_motor_type2raw_motor_type(
            &udp_receive_data.udp_motor_send[i * 3 + 2],
            &leg[i].knee_motor.command);
      }
    }
    // Update UDP remote IP and port (not needed for HexapodSoftware udp_comm)
    // upcb->remote_ip = *addr;  // 璁板綍杩滅▼涓绘満鐨処P鍦板潃
    // upcb->remote_port = port; // 璁板綍杩滅▼涓绘満鐨勭鍙ｅ彿
    // lwipdev.remoteip[0] = upcb->remote_ip.addr & 0xff;         // IADDR4
    // lwipdev.remoteip[1] = (upcb->remote_ip.addr >> 8) & 0xff;  // IADDR3
    // lwipdev.remoteip[2] = (upcb->remote_ip.addr >> 16) & 0xff; // IADDR2
    // lwipdev.remoteip[3] = (upcb->remote_ip.addr >> 24) & 0xff; // IADDR1
    udp_demo_flag |= 1 << 6; // 鏍囪鎺ユ敹鍒版暟鎹簡
    pbuf_free(p);            // 閲婃斁鍐呭瓨
  }
  // else {
  //   udp_disconnect(upcb);
  //   udp_demo_flag &= ~(1 << 5); // 鏍囪杩炴帴鏂紑
  // }

  // send motor feedback to PC
  udp_connect(upcb, addr, UDP_REMOTE_PORT);
  udp_demo_senddata(upcb);
  // IMPORTANT: free the connection to accept new clients
  udp_disconnect(upcb);
}

// UDP鏈嶅姟鍣ㄥ彂閫佹暟鎹�
int len_udp_send = 0;
void udp_demo_senddata(struct udp_pcb *upcb) {
  SCB_CleanDCache();
  len_udp_send = sizeof(udp_send_data);
  struct pbuf *ptr;
  ptr = pbuf_alloc(PBUF_TRANSPORT, sizeof(udp_send_data), PBUF_POOL);
  if (ptr) {
    // 灏唗cp_demo_sendbuf涓殑鏁版嵁鎵撳寘杩沺buf缁撴瀯涓�
    pbuf_take(ptr, (char *)&udp_send_data, sizeof(udp_send_data));
    udp_send(upcb, ptr);
    pbuf_free(ptr);
  }
}

// 鍏抽棴UDP杩炴帴
void udp_demo_connection_close(struct udp_pcb *upcb) {
  udp_disconnect(upcb);
  udp_remove(upcb);           // 鏂紑UDP杩炴帴
  udp_demo_flag &= ~(1 << 5); // 鏍囪杩炴帴鏂紑
}
