#ifndef __UDP_DEMO_H
#define __UDP_DEMO_H
#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "lwip/udp.h"
#include "lwip_comm.h"
#include "sys.h"

#define UDP_DEMO_RX_BUFSIZE 2000 // 瀹氫箟udp鏈€澶ф帴鏀舵暟鎹暱搴�
#define UDP_LOCAL_PORT 10
#define UDP_REMOTE_PORT 7

void udp_demo_test(void);
void udp_demo_recv(void *arg, struct udp_pcb *upcb, struct pbuf *p,
                   struct ip_addr *addr, u16_t port);
void udp_demo_senddata(struct udp_pcb *upcb);
void udp_demo_connection_close(struct udp_pcb *upcb);

#endif
