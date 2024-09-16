#include "sys.h"
#include "delay.h"
#include "usart.h" 
#include "led.h" 		 	 
#include "lcd.h" 	 
#include "gps.h"	
#include "usart3.h" 	
#include "key.h" 	 
#include "string.h"	


u8 USART1_TX_BUF[USART3_MAX_RECV_LEN]; 					//����1,���ͻ�����
nmea_msg gpsx; 											//GPS��Ϣ
__align(4) u8 dtbuf[50];   								//��ӡ������
const u8*fixmode_tbl[4]={"Fail","Fail"," 2D "," 3D "};	//fix mode�ַ��� 
	   

int main(void)
{	 
	u16 i,rxlen;
	u8 key=0XFF;
   
	Power_switch();		//�����ܵ�Դ����
	Power_ON();
	
	delay_init();	    	 //��ʱ������ʼ��	  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�����ж����ȼ�����Ϊ��2��2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);		//����PWR��ʱ��
	
	OLED_Init();
	MyRTC_Init();
	
	MAX6675_GPIO_Init();	//��ʼ���ȵ�ż
	uart_init(115200);	 	//���ڳ�ʼ��Ϊ115200
 	LED_Init();		  			//��ʼ����LED���ӵ�Ӳ���ӿ�
	KEY_Init();					//��ʼ������ 
	usart3_init(38400);		//��ʼ������3 
	Usart2_Init(115200);
	
	uint32_t Alqrm = RTC_GetCounter() + 60*60;			//�����趨1Сʱ����һ��
	RTC_SetAlarm(Alqrm);
 
	Test_GPS();		//���ö�λ��Ϣ�����ٶ�Ϊ5Hz,˳���ж�GPSģ���Ƿ���λ. 
	
	while(1) 
	{
		
		int Temperature = MAX6675_Get_Temperature();
		printf("%d\r\n", Temperature);

		if(Temperature >= 60)		//�¶ȹ���
		{
			retry:
			delay_ms(1);
			if(USART3_RX_STA&0X8000)		//���յ�һ��������
			{
				rxlen=USART3_RX_STA&0X7FFF;	//�õ����ݳ���
				for(i=0;i<rxlen;i++)USART1_TX_BUF[i]=USART3_RX_BUF[i];	   
				USART3_RX_STA=0;		   	//������һ�ν���
				USART1_TX_BUF[i]=0;			//�Զ���ӽ�����
				GPS_Analysis(&gpsx,(u8*)USART1_TX_BUF);//�����ַ���	
				Gps_Msg_Show();				//��ʾ��Ϣ	
				
				if(gpsx.fixmode <= 1)		//���û�ж�λ
				{
					goto retry;
				}
			}
			
			sim900a_send();
		}
		
	OLED_ShowString(0, 1, "Longitude:", OLED_6X8);		//����
	OLED_Update();
		OLED_ShowData();
		
		OLED_Clear();
		Power_OFF();
		PWR_EnterSTANDBYMode();		//STM32����ֹͣģʽ�����ȴ�ָ���Ļ����¼���WKUP�����ػ�RTC���ӣ�
	}
		
}


















