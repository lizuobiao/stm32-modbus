#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 

//如果想串口中断接收，请不要注释以下宏定义
void uart_init(u32 bound);
void recv35_timeout(void);
void USART1_Send_Data(u8 *buf,u8 len);
#endif


