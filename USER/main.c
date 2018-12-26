#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "timer.h"
#include "modbus-layout.h"

float adc_fl,temp_fl;
//传感器数据刷新
void temp_adc_update(void)
{
		u16 adc_int = (u16)(adc_fl*1000);
		u16 temp_int = (u16)(temp_fl*1000);
		delay_ms(100);
		set_temp_adc(adc_int,temp_int);
}

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
