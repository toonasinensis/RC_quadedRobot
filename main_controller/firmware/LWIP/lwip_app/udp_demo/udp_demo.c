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

// UDP接收数据缓冲区
u8 udp_demo_recvbuf[UDP_DEMO_RX_BUFSIZE]; // UDP接收数据缓冲区
// UDP发送数据内容
u8 udp_send_buffer_quad[200];
const u8 *tcp_demo_sendbuf = "Apollo STM32F4/F7/H7 UDP demo send data\r\n";

// UDP 测试全局状态标记变量
// bit7:没有用到
// bit6:0,没有收到数据;1,收到数据了.
// bit5:0,没有连接上;1,连接上了.
// bit4~0:保留
u8 udp_demo_flag;

// 设置远端IP地址
extern uint8_t fast_send;
void udp_demo_set_remoteip(void) {
  // u8 *tbuf;
  // u16 xoff;
  // u8 key;

  // tbuf = mymalloc(SRAMIN, 400); // 申请内存
  // if (tbuf == NULL)
  //   return;
  // 前三个IP保持和DHCP得到的IP一致
  lwipdev.remoteip[0] = lwipdev.ip[0];
  lwipdev.remoteip[1] = lwipdev.ip[1];
  lwipdev.remoteip[2] = lwipdev.ip[2];

  // myfree(SRAMIN, tbuf);
}

// UDP测试
u8 udp_send_flag;

void udp_demo_test(void) {
  err_t err;
  struct udp_pcb *udppcb; // 定义一个TCP服务器控制块
  // struct ip_addr rmtipaddr; // 远端ip地址

  u8 *tbuf;
  u8 key;
  u8 res = 0;
  u16 t = 0;

  udp_demo_set_remoteip(); // 先选择IP

  tbuf = mymalloc(SRAMIN, 400); // 申请内存
  if (tbuf == NULL)
    return; // 内存申请失败了,直接退出

  udppcb = udp_new();
  if (udppcb) // 创建成功
  {
    // IP4_ADDR(&rmtipaddr, lwipdev.remoteip[0], lwipdev.remoteip[1],
    //          lwipdev.remoteip[2], lwipdev.remoteip[3]);
    // NOTE: No need to connect right now
    // UDP客户端连接到指定IP地址和端口号的服务器
    // err = udp_connect(udppcb, &rmtipaddr, UDP_REMOTE_PORT);
    err = udp_bind(udppcb, IP_ADDR_ANY, UDP_LOCAL_PORT);
    if (err == ERR_OK) {
      udp_recv(udppcb, udp_demo_recv, NULL); // 注册接收回调函数
      // 标记连接上了(UDP是非可靠连接,这里仅仅表示本地UDP已经准备好)
      udp_demo_flag |= 1 << 5; // 标记已经连接上
    } else
      res = 1;
  } else
    res = 1;

  // Make sure udp is disconnected
  udp_disconnect(udppcb);

  fast_send = 1;
  while (1) {
    // key = KEY_Scan(0);
    // if (key == WKUP_PRES || udp_send_flag == 1) // KEY0按下了,发送数据
    // {
    //   udp_send_flag = 0;
    //   udp_demo_senddata(udppcb);
    // }
    // if (udp_demo_flag & 1 << 6) // 是否收到数据?
    // {
    //   udp_demo_flag &= ~(1 << 6); // 标记数据已经被处理了.
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
extern uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; // 以太网接收缓冲区

int udp_cnt=0;

// UDP回调函数
void udp_demo_recv(void *arg, struct udp_pcb *upcb, struct pbuf *p,
                   struct ip_addr *addr, u16_t port) {
	udp_cnt++;
  SCB_InvalidateDCache();
  u32 data_len = 0;
  u8 success_rev_flag = 0;
  struct pbuf *q;

  // get motor command from PC
  if (p != NULL) // 接收到不为空的数据时
  {
			SCB_InvalidateDCache_by_Addr((uint32_t *)Rx_Buff, ETH_RX_DESC_CNT*ETH_MAX_PACKET_SIZE);

    memset(udp_demo_recvbuf, 0, UDP_DEMO_RX_BUFSIZE); // 数据接收缓冲区清零
    for (q = p; q != NULL; q = q->next) {
      if (q->len > (UDP_DEMO_RX_BUFSIZE - data_len)) {
        memcpy(udp_demo_recvbuf + data_len, q->payload,
               (UDP_DEMO_RX_BUFSIZE - data_len)); // 拷贝数据
      } else {
        memcpy(udp_demo_recvbuf + data_len, q->payload, q->len);
      }
      data_len += q->len;
      if (data_len > UDP_DEMO_RX_BUFSIZE)
        break; // 超出TCP客户端接收数组,跳出
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
    // upcb->remote_ip = *addr;  // 记录远程主机的IP地址
    // upcb->remote_port = port; // 记录远程主机的端口号
    // lwipdev.remoteip[0] = upcb->remote_ip.addr & 0xff;         // IADDR4
    // lwipdev.remoteip[1] = (upcb->remote_ip.addr >> 8) & 0xff;  // IADDR3
    // lwipdev.remoteip[2] = (upcb->remote_ip.addr >> 16) & 0xff; // IADDR2
    // lwipdev.remoteip[3] = (upcb->remote_ip.addr >> 24) & 0xff; // IADDR1
    udp_demo_flag |= 1 << 6; // 标记接收到数据了
    pbuf_free(p);            // 释放内存
  }
  // else {
  //   udp_disconnect(upcb);
  //   udp_demo_flag &= ~(1 << 5); // 标记连接断开
  // }

  // send motor feedback to PC
  udp_connect(upcb, addr, UDP_REMOTE_PORT);
  udp_demo_senddata(upcb);
  // IMPORTANT: free the connection to accept new clients
  udp_disconnect(upcb);
}

// UDP服务器发送数据
int len_udp_send = 0;
void udp_demo_senddata(struct udp_pcb *upcb) {
  SCB_CleanDCache();
  len_udp_send = sizeof(udp_send_data);
  struct pbuf *ptr;
  ptr = pbuf_alloc(PBUF_TRANSPORT, sizeof(udp_send_data), PBUF_POOL);
  if (ptr) {
    // 将tcp_demo_sendbuf中的数据打包进pbuf结构中
    pbuf_take(ptr, (char *)&udp_send_data, sizeof(udp_send_data));
    udp_send(upcb, ptr);
    pbuf_free(ptr);
  }
}

// 关闭UDP连接
void udp_demo_connection_close(struct udp_pcb *upcb) {
  udp_disconnect(upcb);
  udp_remove(upcb);           // 断开UDP连接
  udp_demo_flag &= ~(1 << 5); // 标记连接断开
}
