#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "timer.h"
#include "lcd.h"
#include "key.h"
#include "beep.h"
#include "malloc.h"
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "freertos_demo.h"


extern uint8_t Su03t_Data;

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//����ϵͳ�ж����ȼ�����4	 
	delay_init();	    				//��ʱ������ʼ��	 
	uart_init(115200);					//��ʼ������
	LED_Init();		  					//��ʼ��LED
	KEY_Init();							//��ʼ������
	BEEP_Init();						//��ʼ��������
	my_mem_init(SRAMIN);            	//��ʼ���ڲ��ڴ��
	
	//����Ҫ���볬����ģ�飬�������У���Ϊ��ֵ�ź��Ǵ������������������
	Distance_Init();		
	USART2_Init(115200);
	USART3_Init(115200);
	Motor_PWM_Init(100-1, 7200-1);
	OLED_Init();
	SG90_Init();
	
	
	while(1)
	{
		uint8_t key = KEY_Scan(0);
		if(Su03t_Data == 1 || key == WKUP_PRES) {break;}
	}
	
	
	freertos_demo();
}




