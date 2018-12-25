#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "timer.h"
#include "modbus.h"

int main(void)
{
	delay_init();	    	 //延时函数初始化	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uart_init(9600);	 //串口初始化为115200
	TIM3_Int_Init(9,7199);//10Khz的计数频率，计数到5000为500ms  

 	while(1)
	{
//		USART1_Send_Data("hello world\r\n",5);
		md_pkt_deal();
		delay_ms(1);
	}	 
}
