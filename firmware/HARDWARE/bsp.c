/*
 * bsp.c
 *
 *  Created on: Apr 22, 2021
 *      Author: Administrator
 */
#include "bsp.h"

/* ���غ� */

#define USING_RTX5 0  /* ʹ���������RTX5��Ƕ��ʽʵʱϵͳ�� */

/**
  * ��������: �������쳣����ӡ���쳣��λ�ã��ļ������кţ�
  * �������: file: Դ�����ļ����ơ��ؼ���__FILE__��ʾԴ�����ļ�����
  *        line: �����кţ��ؼ���__LINE__��ʾԴ�����к�
   * ����ֵ��  void

  * ˵��:
  *    1������Ҫ���Եĵط�ֱ�ӵ���user_Assert(__FILE__,__LINE__);
  */
void user_Assert(char *file,uint32_t line)
{
	printf("Wrong parameters value: file %s on line %d\r\n",file, (unsigned int)line);

    /* ����һ����ѭ��������ʧ��ʱ����Ῠ�˴��������Ա����û���� */
	if(line == 0)
	{
		return;
	}
	while(1)
	{
#if USING_RTX5
		 osKernelLock();  /* �������̵߳ĵ��ȹر� */
		
		 printf("Wrong parameters value: file %s on line %d\r\n",file, (unsigned int)line);
		 HAL_Delay(300);
	
#else		
		 printf("Wrong parameters value: file %s on line %d\r\n",file, (unsigned int)line);
		 HAL_Delay(300);
		
#endif
	}
}









