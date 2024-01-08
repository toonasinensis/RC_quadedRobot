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

// UDP�������ݻ�����
u8 udp_demo_recvbuf[UDP_DEMO_RX_BUFSIZE]; // UDP�������ݻ�����
// UDP������������
u8 udp_send_buffer_quad[200];
const u8 *tcp_demo_sendbuf = "Apollo STM32F4/F7/H7 UDP demo send data\r\n";

// UDP ����ȫ��״̬��Ǳ���
// bit7:û���õ�
// bit6:0,û���յ�����;1,�յ�������.
// bit5:0,û��������;1,��������.
// bit4~0:����
u8 udp_demo_flag;

// ����Զ��IP��ַ
void udp_demo_set_remoteip(void) {
  u8 *tbuf;
  u16 xoff;
  u8 key;

  tbuf = mymalloc(SRAMIN, 400); // �����ڴ�
  if (tbuf == NULL)
    return;
  // ǰ����IP���ֺ�DHCP�õ���IPһ��
  lwipdev.remoteip[0] = lwipdev.ip[0];
  lwipdev.remoteip[1] = lwipdev.ip[1];
  lwipdev.remoteip[2] = lwipdev.ip[2];

  myfree(SRAMIN, tbuf);
}

// UDP����
u8 udp_send_flag;

void udp_demo_test(void) {
  err_t err;
  struct udp_pcb *udppcb;   // ����һ��TCP���������ƿ�
  struct ip_addr rmtipaddr; // Զ��ip��ַ

  u8 *tbuf;
  u8 key;
  u8 res = 0;
  u16 t = 0;

  udp_demo_set_remoteip(); // ��ѡ��IP

  tbuf = mymalloc(SRAMIN, 400); // �����ڴ�
  if (tbuf == NULL)
    return; // �ڴ�����ʧ����,ֱ���˳�

  udppcb = udp_new();
  if (udppcb) // �����ɹ�
  {
    IP4_ADDR(&rmtipaddr, lwipdev.remoteip[0], lwipdev.remoteip[1],
             lwipdev.remoteip[2], lwipdev.remoteip[3]);
    err = udp_connect(
        udppcb, &rmtipaddr,
        UDP_REMOTE_PORT); // UDP�ͻ������ӵ�ָ��IP��ַ�Ͷ˿ںŵķ�����
    if (err == ERR_OK) {
      err = udp_bind(udppcb, IP_ADDR_ANY,
                     UDP_LOCAL_PORT); // �󶨱���IP��ַ��˿ں�
      if (err == ERR_OK)              // �����
      {
        udp_recv(udppcb, udp_demo_recv, NULL); // ע����ջص�����
        //				LCD_ShowString(30,210,210,16,16,"STATUS:Connected
        //");//�����������(UDP�Ƿǿɿ�����,���������ʾ����UDP�Ѿ�׼����)
        udp_demo_flag |= 1 << 5; // ����Ѿ�������

      } else
        res = 1;
    } else
      res = 1;
  } else
    res = 1;

  while (1) {
    key = KEY_Scan(0);

    if (key == WKUP_PRES || udp_send_flag == 1) // KEY0������,��������
    {
      udp_send_flag = 0;
      udp_demo_senddata(udppcb);
    }
    if (udp_demo_flag & 1 << 6) // �Ƿ��յ�����?
    {
      udp_demo_flag &= ~(1 << 6); // ��������Ѿ���������.
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

// UDP�ص�����
void udp_demo_recv(void *arg, struct udp_pcb *upcb, struct pbuf *p,
                   struct ip_addr *addr, u16_t port) {
  SCB_InvalidateDCache();

  u32 data_len = 0;
  u8 success_rev_flag = 0;
  struct pbuf *q;
  if (p != NULL) // ���յ���Ϊ�յ�����ʱ
  {
    memset(udp_demo_recvbuf, 0, UDP_DEMO_RX_BUFSIZE); // ���ݽ��ջ���������
    for (q = p; q != NULL; q = q->next) // ����������pbuf����
    {
      // �ж�Ҫ������UDP_DEMO_RX_BUFSIZE�е������Ƿ����UDP_DEMO_RX_BUFSIZE��ʣ��ռ䣬�������
      // �Ļ���ֻ����UDP_DEMO_RX_BUFSIZE��ʣ�೤�ȵ����ݣ�����Ļ��Ϳ������е�����
      if (q->len > (UDP_DEMO_RX_BUFSIZE - data_len)) {
        memcpy(udp_demo_recvbuf + data_len, q->payload,
               (UDP_DEMO_RX_BUFSIZE - data_len)); // ��������
      } else {
        memcpy(udp_demo_recvbuf + data_len, q->payload, q->len);
      }

      data_len += q->len;
      if (data_len > UDP_DEMO_RX_BUFSIZE)
        break; // ����TCP�ͻ��˽�������,����
    }
    upcb->remote_ip = *addr;  // ��¼Զ��������IP��ַ
    upcb->remote_port = port; // ��¼Զ�������Ķ˿ں�
    lwipdev.remoteip[0] = upcb->remote_ip.addr & 0xff;         // IADDR4
    lwipdev.remoteip[1] = (upcb->remote_ip.addr >> 8) & 0xff;  // IADDR3
    lwipdev.remoteip[2] = (upcb->remote_ip.addr >> 16) & 0xff; // IADDR2
    lwipdev.remoteip[3] = (upcb->remote_ip.addr >> 24) & 0xff; // IADDR1
    udp_demo_flag |= 1 << 6; // ��ǽ��յ�������
    pbuf_free(p);            // �ͷ��ڴ�

  } else {
    udp_disconnect(upcb);
    udp_demo_flag &= ~(1 << 5); // ������ӶϿ�
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
// UDP��������������
int len_udp_send = 0;
void udp_demo_senddata(struct udp_pcb *upcb) {
  SCB_CleanDCache();
  len_udp_send = sizeof(udp_send_data);
  struct pbuf *ptr;
  ptr = pbuf_alloc(PBUF_TRANSPORT, sizeof(udp_send_data), PBUF_POOL); // �����ڴ�
  if (ptr) {
    pbuf_take(
        ptr, (char *)&udp_send_data,
        sizeof(udp_send_data)); // ��tcp_demo_sendbuf�е����ݴ����pbuf�ṹ��
    udp_send(upcb, ptr); // udp��������
    pbuf_free(ptr);      // �ͷ��ڴ�
  }
}
// �ر�UDP����
void udp_demo_connection_close(struct udp_pcb *upcb) {
  udp_disconnect(upcb);
  udp_remove(upcb);           // �Ͽ�UDP����
  udp_demo_flag &= ~(1 << 5); // ������ӶϿ�
}

// void udp_demo_test(void)
//{
//  	err_t err;
//	struct udp_pcb *udppcb;  	//����һ��TCP���������ƿ�
//	struct ip_addr rmtipaddr;  	//Զ��ip��ַ
//
//	u8 *tbuf;
//  	u8 key;
//	u8 res=0;
//	u16 t=0;
//
//	udp_demo_set_remoteip();//��ѡ��IP

//	tbuf=mymalloc(SRAMIN,200);	//�����ڴ�
//	if(tbuf==NULL)return ;		//�ڴ�����ʧ����,ֱ���˳�

//	udppcb=udp_new();
//	if(udppcb)//�����ɹ�
//	{
//		IP4_ADDR(&rmtipaddr,lwipdev.remoteip[0],lwipdev.remoteip[1],lwipdev.remoteip[2],lwipdev.remoteip[3]);
//		err=udp_connect(udppcb,&rmtipaddr,UDP_DEMO_PORT);//UDP�ͻ������ӵ�ָ��IP��ַ�Ͷ˿ںŵķ�����
//		if(err==ERR_OK)
//		{
//			err=udp_bind(udppcb,IP_ADDR_ANY,UDP_DEMO_PORT);//�󶨱���IP��ַ��˿ں�
//			if(err==ERR_OK)	//�����
//			{
//				udp_recv(udppcb,udp_demo_recv,NULL);//ע����ջص�����
////				LCD_ShowString(30,210,210,16,16,"STATUS:Connected
///");//�����������(UDP�Ƿǿɿ�����,���������ʾ����UDP�Ѿ�׼����)
//				udp_demo_flag |= 1<<5;
////����Ѿ�������

//			}else res=1;
//		}else res=1;
//	}else res=1;
//	while(res==0)
//	{
//		key=KEY_Scan(0);
//		if(key==WKUP_PRES)break;
//		if(key==KEY0_PRES)//KEY0������,��������
//		{
//			udp_demo_senddata(udppcb);
//		}
//		if(udp_demo_flag&1<<6)//�Ƿ��յ�����?
//		{
////
///LCD_Fill(30,250,lcddev.width-1,lcddev.height-1,WHITE);//����һ������ /
///LCD_ShowString(30,250,lcddev.width-30,lcddev.height-230,16,udp_demo_recvbuf);//��ʾ���յ�������
//			udp_demo_flag&=~(1<<6);//��������Ѿ���������.
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
