#include "lwip_comm.h"
#include "delay.h"
#include "ethernetif.h"
#include "lwip/dhcp.h"
#include "lwip/init.h"
#include "lwip/ip_frag.h"
#include "lwip/mem.h"
#include "lwip/memp.h"
#include "lwip/tcp_impl.h"
#include "lwip/tcpip.h"
#include "lwip/timers.h"
#include "malloc.h"
#include "netif/etharp.h"
#include "pcf8574.h"
#include "usart.h"
#include <stdio.h>

//////////////////////////////////////////////////////////////////////////////////
// 本程序只供学习使用，未经作者许可，不得用于其它任何用途
// ALIENTEK STM32H7开发板
// lwip通用驱动 代码
// 正点原子@ALIENTEK
// 技术论坛:www.openedv.com
// 创建日期:2018/7/13
// 版本：V1.0
// 版权所有，盗版必究。
// Copyright(C) 广州市星翼电子科技有限公司 2009-2019
// All rights reserved
//*******************************************************************************
// 修改信息
// 无
//////////////////////////////////////////////////////////////////////////////////

__lwip_dev lwipdev;      // lwip控制结构体
struct netif lwip_netif; // 定义一个全局的网络接口

u32 TCPTimer = 0;   // TCP查询计时器
u32 ARPTimer = 0;   // ARP查询计时器
u32 lwip_localtime; // lwip本地时间计数器,单位:ms

#if LWIP_DHCP
u32 DHCPfineTimer = 0;   // DHCP精细处理计时器
u32 DHCPcoarseTimer = 0; // DHCP粗糙处理计时器
#endif

// lwip 默认IP设置
// lwipx:lwip控制结构体指针
void lwip_comm_default_ip_set(__lwip_dev *lwipx) {
  u32 sn0;
  sn0 = *(vu32 *)(0x1FF1E800); // 获取STM32的唯一ID的前24位作为MAC地址后三字节
  // 默认远端IP为:192.168.1.67
  lwipx->remoteip[0] = 192;
  lwipx->remoteip[1] = 168;
  lwipx->remoteip[2] =1 ;
  lwipx->remoteip[3] = 102;
  // MAC地址设置(高三字节固定为:2.0.0,低三字节用STM32唯一ID)
  lwipx->mac[0] = 2; // 高三字节(IEEE称之为组织唯一ID,OUI)地址固定为:2.0.0
  lwipx->mac[1] = 0;
  lwipx->mac[2] = 0;
  lwipx->mac[3] = (sn0 >> 16) & 0XFF; // 低三字节用STM32的唯一ID
  lwipx->mac[4] = (sn0 >> 8) & 0XFFF;
  lwipx->mac[5] = sn0 & 0XFF;
  // 默认本地IP为:192.168.1.30
  lwipx->ip[0] = 192;
  lwipx->ip[1] = 168;
  lwipx->ip[2] = 1;
  lwipx->ip[3] = 10;
  // 默认子网掩码:255.255.255.0
  lwipx->netmask[0] = 255;
  lwipx->netmask[1] = 255;
  lwipx->netmask[2] = 255;
  lwipx->netmask[3] = 0;
  // 默认网关:192.168.1.1
  lwipx->gateway[0] = 192;
  lwipx->gateway[1] = 168;
  lwipx->gateway[2] = 1;
  lwipx->gateway[3] = 1;
  lwipx->dhcpstatus = 0; // 没有DHCP
}

// LWIP初始化(LWIP启动的时候使用)
// 返回值:0,成功
//       1,内存错误
//       2,LAN8720初始化失败
//       3,网卡添加失败.
u8 lwip_comm_init(void) {
  u8 retry = 0;
  struct netif *
      Netif_Init_Flag; // 调用netif_add()函数时的返回值,用于判断网络初始化是否成功
  struct ip_addr ipaddr;  // ip地址
  struct ip_addr netmask; // 子网掩码
  struct ip_addr gw;      // 默认网关

  lwip_comm_default_ip_set(&lwipdev); // 设置默认IP等信
  while (LAN8720_Init()) // 初始化LAN8720,如果失败的话就重试5次
  {
    retry++;
    if (retry > 5) {
      retry = 0;
      return 3;
    } // LAN8720初始化失败
  }
  lwip_init(); // 初始化LWIP内核

#if LWIP_DHCP // 使用动态IP
  ipaddr.addr = 0;
  netmask.addr = 0;
  gw.addr = 0;
#else // 使用静态IP
  IP4_ADDR(&ipaddr, lwipdev.ip[0], lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3]);
  IP4_ADDR(&netmask, lwipdev.netmask[0], lwipdev.netmask[1], lwipdev.netmask[2],
           lwipdev.netmask[3]);
  IP4_ADDR(&gw, lwipdev.gateway[0], lwipdev.gateway[1], lwipdev.gateway[2],
           lwipdev.gateway[3]);
  printf("网卡en的MAC地址为:................%d.%d.%d.%d.%d.%d\r\n",
         lwipdev.mac[0], lwipdev.mac[1], lwipdev.mac[2], lwipdev.mac[3],
         lwipdev.mac[4], lwipdev.mac[5]);
  printf("静态IP地址........................%d.%d.%d.%d\r\n", lwipdev.ip[0],
         lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3]);
  printf("子网掩码..........................%d.%d.%d.%d\r\n",
         lwipdev.netmask[0], lwipdev.netmask[1], lwipdev.netmask[2],
         lwipdev.netmask[3]);
  printf("默认网关..........................%d.%d.%d.%d\r\n",
         lwipdev.gateway[0], lwipdev.gateway[1], lwipdev.gateway[2],
         lwipdev.gateway[3]);
#endif
  Netif_Init_Flag =
      netif_add(&lwip_netif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init,
                &ethernet_input); // 向网卡列表中添加一个网口

#if LWIP_DHCP              // 如果使用DHCP的话
  lwipdev.dhcpstatus = 0;  // DHCP标记为0
  dhcp_start(&lwip_netif); // 开启DHCP服务
#endif

  if (Netif_Init_Flag == NULL)
    return 4; // 网卡添加失败
  else // 网口添加成功后,设置netif为默认值,并且打开netif网口
  {
    netif_set_default(&lwip_netif); // 设置netif为默认网口
    netif_set_up(&lwip_netif);      // 打开netif网口
  }
  return 0; // 操作OK.
}

// 当接收到数据后调用
void lwip_pkt_handle(void) {
  // 从网络缓冲区中读取接收到的数据包并将其发送给LWIP处理
  ethernetif_input(&lwip_netif);
}

// LWIP轮询任务
void lwip_periodic_handle(void) {
  sys_check_timeouts();
#if LWIP_DHCP // 如果使用DHCP的话
  // 每500ms调用一次dhcp_fine_tmr()
  if (lwip_localtime - DHCPfineTimer >= DHCP_FINE_TIMER_MSECS) {
    DHCPfineTimer = lwip_localtime;
    dhcp_fine_tmr();
    if ((lwipdev.dhcpstatus != 2) && (lwipdev.dhcpstatus != 0XFF)) {
      lwip_dhcp_process_handle(); // DHCP处理
    }
  }

  // 每60s执行一次DHCP粗糙处理
  if (lwip_localtime - DHCPcoarseTimer >= DHCP_COARSE_TIMER_MSECS) {
    DHCPcoarseTimer = lwip_localtime;
    dhcp_coarse_tmr();
  }
#endif
}

// 如果使能了DHCP
#if LWIP_DHCP

// DHCP处理任务
void lwip_dhcp_process_handle(void) {
  u32 ip = 0, netmask = 0, gw = 0;
  switch (lwipdev.dhcpstatus) {
  case 0: // 开启DHCP
    dhcp_start(&lwip_netif);
    lwipdev.dhcpstatus = 1; // 等待通过DHCP获取到的地址
    printf("正在查找DHCP服务器,请稍等...........\r\n");
    break;
  case 1: // 等待获取到IP地址
  {
    ip = lwip_netif.ip_addr.addr;      // 读取新IP地址
    netmask = lwip_netif.netmask.addr; // 读取子网掩码
    gw = lwip_netif.gw.addr;           // 读取默认网关

    if (ip != 0) // 正确获取到IP地址的时候
    {
      lwipdev.dhcpstatus = 2; // DHCP成功
      printf("网卡en的MAC地址为:................%d.%d.%d.%d.%d.%d\r\n",
             lwipdev.mac[0], lwipdev.mac[1], lwipdev.mac[2], lwipdev.mac[3],
             lwipdev.mac[4], lwipdev.mac[5]);
      // 解析出通过DHCP获取到的IP地址
      lwipdev.ip[3] = (uint8_t)(ip >> 24);
      lwipdev.ip[2] = (uint8_t)(ip >> 16);
      lwipdev.ip[1] = (uint8_t)(ip >> 8);
      lwipdev.ip[0] = (uint8_t)(ip);
      printf("通过DHCP获取到IP地址..............%d.%d.%d.%d\r\n", lwipdev.ip[0],
             lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3]);
      // 解析通过DHCP获取到的子网掩码地址
      lwipdev.netmask[3] = (uint8_t)(netmask >> 24);
      lwipdev.netmask[2] = (uint8_t)(netmask >> 16);
      lwipdev.netmask[1] = (uint8_t)(netmask >> 8);
      lwipdev.netmask[0] = (uint8_t)(netmask);
      printf("通过DHCP获取到子网掩码............%d.%d.%d.%d\r\n",
             lwipdev.netmask[0], lwipdev.netmask[1], lwipdev.netmask[2],
             lwipdev.netmask[3]);
      // 解析出通过DHCP获取到的默认网关
      lwipdev.gateway[3] = (uint8_t)(gw >> 24);
      lwipdev.gateway[2] = (uint8_t)(gw >> 16);
      lwipdev.gateway[1] = (uint8_t)(gw >> 8);
      lwipdev.gateway[0] = (uint8_t)(gw);
      printf("通过DHCP获取到的默认网关..........%d.%d.%d.%d\r\n",
             lwipdev.gateway[0], lwipdev.gateway[1], lwipdev.gateway[2],
             lwipdev.gateway[3]);
    } else if (
        lwip_netif.dhcp->tries >
        LWIP_MAX_DHCP_TRIES) // 通过DHCP服务获取IP地址失败,且超过最大尝试次数
    {
      lwipdev.dhcpstatus = 0XFF; // DHCP超时失败.
      // 使用静态IP地址
      IP4_ADDR(&(lwip_netif.ip_addr), lwipdev.ip[0], lwipdev.ip[1],
               lwipdev.ip[2], lwipdev.ip[3]);
      IP4_ADDR(&(lwip_netif.netmask), lwipdev.netmask[0], lwipdev.netmask[1],
               lwipdev.netmask[2], lwipdev.netmask[3]);
      IP4_ADDR(&(lwip_netif.gw), lwipdev.gateway[0], lwipdev.gateway[1],
               lwipdev.gateway[2], lwipdev.gateway[3]);
      printf("DHCP服务超时,使用静态IP地址!\r\n");
      printf("网卡en的MAC地址为:................%d.%d.%d.%d.%d.%d\r\n",
             lwipdev.mac[0], lwipdev.mac[1], lwipdev.mac[2], lwipdev.mac[3],
             lwipdev.mac[4], lwipdev.mac[5]);
      printf("静态IP地址........................%d.%d.%d.%d\r\n", lwipdev.ip[0],
             lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3]);
      printf("子网掩码..........................%d.%d.%d.%d\r\n",
             lwipdev.netmask[0], lwipdev.netmask[1], lwipdev.netmask[2],
             lwipdev.netmask[3]);
      printf("默认网关..........................%d.%d.%d.%d\r\n",
             lwipdev.gateway[0], lwipdev.gateway[1], lwipdev.gateway[2],
             lwipdev.gateway[3]);
    }
  } break;
  default:
    break;
  }
}
#endif
