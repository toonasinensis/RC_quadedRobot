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
void udp_demo_set_remoteip(void) {
  u8 *tbuf;
  u16 xoff;
  u8 key;

  tbuf = mymalloc(SRAMIN, 400); // 申请内存
  if (tbuf == NULL)
    return;
  // 前三个IP保持和DHCP得到的IP一致
  lwipdev.remoteip[0] = lwipdev.ip[0];
  lwipdev.remoteip[1] = lwipdev.ip[1];
  lwipdev.remoteip[2] = lwipdev.ip[2];

  myfree(SRAMIN, tbuf);
}

// UDP测试
u8 udp_send_flag;

void udp_demo_test(void) {
  err_t err;
  struct udp_pcb *udppcb;   // 定义一个TCP服务器控制块
  struct ip_addr rmtipaddr; // 远端ip地址

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
    IP4_ADDR(&rmtipaddr, lwipdev.remoteip[0], lwipdev.remoteip[1],
             lwipdev.remoteip[2], lwipdev.remoteip[3]);
    err = udp_connect(
        udppcb, &rmtipaddr,
        UDP_REMOTE_PORT); // UDP客户端连接到指定IP地址和端口号的服务器
    if (err == ERR_OK) {
      err = udp_bind(udppcb, IP_ADDR_ANY,
                     UDP_LOCAL_PORT); // 绑定本地IP地址与端口号
      if (err == ERR_OK)              // 绑定完成
      {
        udp_recv(udppcb, udp_demo_recv, NULL); // 注册接收回调函数
        //				LCD_ShowString(30,210,210,16,16,"STATUS:Connected
        //");//标记连接上了(UDP是非可靠连接,这里仅仅表示本地UDP已经准备好)
        udp_demo_flag |= 1 << 5; // 标记已经连接上

      } else
        res = 1;
    } else
      res = 1;
  } else
    res = 1;

  while (1) {
    key = KEY_Scan(0);

    if (key == WKUP_PRES || udp_send_flag == 1) // KEY0按下了,发送数据
    {
      udp_send_flag = 0;
      udp_demo_senddata(udppcb);
    }
    if (udp_demo_flag & 1 << 6) // 是否收到数据?
    {
      udp_demo_flag &= ~(1 << 6); // 标记数据已经被处理了.
    }
    lwip_periodic_handle();
    delay_us(100);
    t++;
    if (t == 2000) {
      t = 0;
      LED0_Toggle;
    }
  }
  udp_demo_connection_close(udppcb);
  myfree(SRAMIN, tbuf);
}

// UDP回调函数
void udp_demo_recv(void *arg, struct udp_pcb *upcb, struct pbuf *p,
                   struct ip_addr *addr, u16_t port) {
  SCB_InvalidateDCache();

  u32 data_len = 0;
  u8 success_rev_flag = 0;
  struct pbuf *q;
  if (p != NULL) // 接收到不为空的数据时
  {
    memset(udp_demo_recvbuf, 0, UDP_DEMO_RX_BUFSIZE); // 数据接收缓冲区清零
    for (q = p; q != NULL; q = q->next) // 遍历完整个pbuf链表
    {
      // 判断要拷贝到UDP_DEMO_RX_BUFSIZE中的数据是否大于UDP_DEMO_RX_BUFSIZE的剩余空间，如果大于
      // 的话就只拷贝UDP_DEMO_RX_BUFSIZE中剩余长度的数据，否则的话就拷贝所有的数据
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
    upcb->remote_ip = *addr;  // 记录远程主机的IP地址
    upcb->remote_port = port; // 记录远程主机的端口号
    lwipdev.remoteip[0] = upcb->remote_ip.addr & 0xff;         // IADDR4
    lwipdev.remoteip[1] = (upcb->remote_ip.addr >> 8) & 0xff;  // IADDR3
    lwipdev.remoteip[2] = (upcb->remote_ip.addr >> 16) & 0xff; // IADDR2
    lwipdev.remoteip[3] = (upcb->remote_ip.addr >> 24) & 0xff; // IADDR1
    udp_demo_flag |= 1 << 6; // 标记接收到数据了
    pbuf_free(p);            // 释放内存

  } else {
    udp_disconnect(upcb);
    udp_demo_flag &= ~(1 << 5); // 标记连接断开
  }

  memcpy(&udp_receive_data, udp_demo_recvbuf, sizeof(udp_receive_data));
  if (crc32_core((uint8_t *)&udp_receive_data,
                 sizeof(udp_receive_data) / 4 - 1) ==
      udp_receive_data.check_digit) {
    udp_send_flag = 1;

    for (int i = 0; i < 6; ++i) {
      udp_motor_type2raw_motor_type(&udp_receive_data.udp_motor_send[i * 3],
                                    &leg[i].hip_motor.command);
      udp_motor_type2raw_motor_type(&udp_receive_data.udp_motor_send[i * 3 + 1],
                                    &leg[i].thigh_motor.command);
      udp_motor_type2raw_motor_type(&udp_receive_data.udp_motor_send[i * 3 + 2],
                                    &leg[i].knee_motor.command);
    }
  }
}
// UDP服务器发送数据
int len_udp_send = 0;
void udp_demo_senddata(struct udp_pcb *upcb) {
  SCB_CleanDCache();
  len_udp_send = sizeof(udp_send_data);
  struct pbuf *ptr;
  ptr = pbuf_alloc(PBUF_TRANSPORT, sizeof(udp_send_data), PBUF_POOL); // 申请内存
  if (ptr) {
    pbuf_take(
        ptr, (char *)&udp_send_data,
        sizeof(udp_send_data)); // 将tcp_demo_sendbuf中的数据打包进pbuf结构中
    udp_send(upcb, ptr); // udp发送数据
    pbuf_free(ptr);      // 释放内存
  }
}
// 关闭UDP连接
void udp_demo_connection_close(struct udp_pcb *upcb) {
  udp_disconnect(upcb);
  udp_remove(upcb);           // 断开UDP连接
  udp_demo_flag &= ~(1 << 5); // 标记连接断开
}

// void udp_demo_test(void)
//{
//  	err_t err;
//	struct udp_pcb *udppcb;  	//定义一个TCP服务器控制块
//	struct ip_addr rmtipaddr;  	//远端ip地址
//
//	u8 *tbuf;
//  	u8 key;
//	u8 res=0;
//	u16 t=0;
//
//	udp_demo_set_remoteip();//先选择IP

//	tbuf=mymalloc(SRAMIN,200);	//申请内存
//	if(tbuf==NULL)return ;		//内存申请失败了,直接退出

//	udppcb=udp_new();
//	if(udppcb)//创建成功
//	{
//		IP4_ADDR(&rmtipaddr,lwipdev.remoteip[0],lwipdev.remoteip[1],lwipdev.remoteip[2],lwipdev.remoteip[3]);
//		err=udp_connect(udppcb,&rmtipaddr,UDP_DEMO_PORT);//UDP客户端连接到指定IP地址和端口号的服务器
//		if(err==ERR_OK)
//		{
//			err=udp_bind(udppcb,IP_ADDR_ANY,UDP_DEMO_PORT);//绑定本地IP地址与端口号
//			if(err==ERR_OK)	//绑定完成
//			{
//				udp_recv(udppcb,udp_demo_recv,NULL);//注册接收回调函数
////				LCD_ShowString(30,210,210,16,16,"STATUS:Connected
///");//标记连接上了(UDP是非可靠连接,这里仅仅表示本地UDP已经准备好)
//				udp_demo_flag |= 1<<5;
////标记已经连接上

//			}else res=1;
//		}else res=1;
//	}else res=1;
//	while(res==0)
//	{
//		key=KEY_Scan(0);
//		if(key==WKUP_PRES)break;
//		if(key==KEY0_PRES)//KEY0按下了,发送数据
//		{
//			udp_demo_senddata(udppcb);
//		}
//		if(udp_demo_flag&1<<6)//是否收到数据?
//		{
////
///LCD_Fill(30,250,lcddev.width-1,lcddev.height-1,WHITE);//清上一次数据 /
///LCD_ShowString(30,250,lcddev.width-30,lcddev.height-230,16,udp_demo_recvbuf);//显示接收到的数据
//			udp_demo_flag&=~(1<<6);//标记数据已经被处理了.
//		}
//		lwip_periodic_handle();
//		delay_us(100);
//		t++;
//		if(t==2000)
//		{
//			t=0;
//			LED0_Toggle;
//		}
//	}
//	udp_demo_connection_close(udppcb);
//	myfree(SRAMIN,tbuf);
//}
