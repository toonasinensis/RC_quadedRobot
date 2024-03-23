/***************************** (C) COPYRIGHT ************************************
* File Name          : udp_demo.h
* Author             : 星光嵌入式
* Version            : V1.0
* Date               : 12/12/2019
* Description        : LWIP UDP应用
* Note               : 版权所有  严禁盗版
********************************************************************************
* 实验平台: 星光猛禽STM32H743VIT6开发板
* 淘宝店铺: https://shop148702745.taobao.com/
* 技术论坛: www.feifanembed.com
* QQ群:542830257
********************************************************************************/
#ifndef __UDP_DEMO_H_
#define __UDP_DEMO_H_

#define UDP_RX_SIZE  512



/*******************************************************************************
* Function Name  : udp_Init
* Description    : lwip的udp初始化
* Input          : None
* Output         : None
* Return         : None
* Note			 : 在回调函数外调用UDP发送函数，都要申请内存空间,udp_Init中的
				   第一、二语句
				   此函数包含了UDP客户端和服务端
				   PC端软件测试时，要指定目标（板子）IP、端口，PC端的端口（1000）
				   PC的IP网段要和板子一致
*******************************************************************************/
void udp_Init(void);

/*******************************************************************************
* Function Name  : UDP_SendData
* Description    : UDP数据发送
* Input          : buff   待发送的数据
                   len    待发送数据长度
* Output         : None
* Return         : None
* Note			 : None
*******************************************************************************/
void UDP_SendData(uint8_t *buff, uint16_t len);
 void udp_demo_senddata(struct udp_pcb *upcb);

#endif
