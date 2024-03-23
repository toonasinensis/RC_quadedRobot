#include "lwip/netif.h"
#include "lwip/ip.h"
#include "lwip/tcp.h"
#include "lwip/init.h"
#include "netif/etharp.h"
#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include <stdio.h>	
#include <string.h>
#include "udp_basic.h"
#include "udp_comm.h"
#include "system_monitor.h"

#define UDP_DEMO_RX_BUFSIZE  350

extern uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_RX_BUFFER_SIZE];

uint8_t udp_recvbuf[UDP_DEMO_RX_BUFSIZE]; // UDP recv buff


struct udp_pcb *UdpPcb;
ip4_addr_t rip_addr; 	//远端IP
	struct pbuf *pb;
void UDP_Receive_Callback(void *arg, struct udp_pcb *upcb, struct pbuf *p,const ip4_addr_t *addr, u16_t port)
{		
	  struct pbuf *q;
  uint32_t data_len = 0;

	if(p != NULL)		/* 如果收到的数据不为空   */ 
	{
			SCB_InvalidateDCache_by_Addr((uint32_t *)Rx_Buff, ETH_RX_DESC_CNT*ETH_MAX_PACKET_SIZE);
    memset(udp_recvbuf, 0, UDP_DEMO_RX_BUFSIZE); // 
		for (q = p; q != NULL; q = q->next) {
      if (q->len > (UDP_DEMO_RX_BUFSIZE - data_len)) {
        memcpy(udp_recvbuf + data_len, q->payload,(UDP_DEMO_RX_BUFSIZE - data_len)); // 
      } else {
        memcpy(udp_recvbuf + data_len, q->payload, q->len);
      }
      data_len += q->len;
      if (data_len > UDP_DEMO_RX_BUFSIZE)
        break; // 
    }
		//receive motor cmd
		  memcpy(&udp_receive_data, udp_recvbuf, sizeof(udp_receive_data));
    if (crc32_core((uint8_t *)&udp_receive_data,
                   sizeof(udp_receive_data) / 4 - 1) ==
        udp_receive_data.check_digit) {
     // LED0_Toggle;
			
					
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
				
		
		pb= p; 							//保存当前控制块
		system_monitor.remote_ctrl_task_cnt++;
		//UDP_SendData((uint8_t *)pb->payload,pb->tot_len);
		UdpPcb->remote_ip = *addr;                  // 获取远端IP地址
		UdpPcb->remote_port = port;                 //记录端口
		udp_demo_senddata(UdpPcb);

		/* Free the p buffer */
		pbuf_free(p);
	}
	else
	{

	}
	
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


void UDP_SendData(uint8_t *buff, uint16_t len)
{
	struct pbuf *p;
	
	p = pbuf_alloc(PBUF_TRANSPORT,strlen((char*)buff), PBUF_POOL);

	if (p != NULL)
	{
		/* copy data to pbuf */
		pbuf_take(p, (char*)buff, strlen((char*)buff));
		  
		/* send udp data */
		udp_send(UdpPcb, p); 

		/* free pbuf */
		pbuf_free(p);
	}
}

extern uint8_t fast_send ;
void udp_Init(void)
{
	err_t err;
	
	/* Create a new UDP control block  */
	UdpPcb = udp_new();      //创建UDP控制块
	
	if(UdpPcb != NULL)
	{
		IP4_ADDR(&rip_addr, 192, 168, 1, 102);	//设置远端IP地址
		err = udp_bind(UdpPcb, IP_ADDR_ANY, 10);	//绑定本地IP地址,端口7
		
		if(err == ERR_OK)   //绑定OK
		{
			err = udp_connect(UdpPcb, &rip_addr, 7);		//连接远程主机,指定其IP和端口

			udp_recv(UdpPcb, UDP_Receive_Callback, NULL);   /* 设置数据接收时的回调函数*/
		fast_send = 1;	
		}
		else
		{
			udp_remove(UdpPcb);     //删除UDP控制块
		}
	}
}

