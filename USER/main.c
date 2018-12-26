#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "timer.h"
#include "modbus-layout.h"

float adc_fl,temp_fl;
//����������ˢ��
void temp_adc_update(void)
{
		u16 adc_int = (u16)(adc_fl*1000);
		u16 temp_int = (u16)(temp_fl*1000);
		delay_ms(100);
		set_temp_adc(adc_int,temp_int);
}

int main(void)
{
	delay_init();	    	 //��ʱ������ʼ��	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart_init(9600);	 //���ڳ�ʼ��Ϊ115200
	TIM3_Int_Init(9,7199);//10Khz�ļ���Ƶ�ʣ�������5000Ϊ500ms  

 	while(1)
	{
//		USART1_Send_Data("hello world\r\n",5);
		md_pkt_deal();
		delay_ms(1);
	}	 
}
