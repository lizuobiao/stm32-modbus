#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 

//����봮���жϽ��գ��벻Ҫע�����º궨��
void uart_init(u32 bound);
void recv35_timeout(void);
void USART1_Send_Data(u8 *buf,u8 len);
#endif


