 #include "sys.h"
#include "delay.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include "sim900a.h"

extern u8 Usart2_buff[128];     /*static��extern������*/
char dispbuf[64];


/**************************************************************************/
//�������ã��ַ���ƥ�Ժ���
//�������ƣ�Find_char(char *a,char *b) ;
//�ڲ�����������a   ����b
//�޸����ڣ�2022��1��26��  �賿2��40
//���ߣ�       (CSDN��)��ƨ��
/**************************************************************************/
u8 Find_char(char *a,char *b)  //bΪ�Ӵ�
{ 
  if(strstr(a,b)!=NULL)
	    return 0;
	else
			return 1;
}

/**************************************************************************/
//�������ã�����Ӣ�Ķ��ź���
//�������ƣ�sim900a_send_English_message(char *message,char *phonenumber)(uint8_t number);
//�ڲ�������message phonenumber
//�޸����ڣ�2022��1��26��  �賿2��40��
//���ߣ�       (CSDN��)��ƨ��
/**************************************************************************/
void sim900a_send_English_message(char *message,char *phonenumber)
{

	Usart_SendString2(USART2,"AT\r\n");                            //SIM900A�Ƿ��뵥Ƭ�������ӳɹ�
	delay_ms(200);	
	while(Find_char((char*)Usart2_buff,"OK"));                      
	printf("English_message_OK1\r\n");
	
	Usart_SendString2(USART2,"AT&F\r\n");                           //SIM900A��λ
	delay_ms(200);	
	while(Find_char((char*)Usart2_buff,"OK"));                     //�ַ���ƥ�Ժ���   
	printf("English_message_OK2\r\n");	
	
	Usart_SendString2(USART2,"AT+CSCS=\"GSM\"\r\n");               //Ӣ�Ķ���ָ��1
	delay_ms(200);	
	while(Find_char((char*)Usart2_buff,"OK"));                     
	printf("English_message_OK3\r\n");
	
	
	Usart_SendString2(USART2,"AT+CMGF=1\r\n");                     //Ӣ�Ķ���ָ��2
	delay_ms(200);	
	while(Find_char((char*)Usart2_buff,"OK"));  
	printf("English_message_OK4\r\n");
	
	sprintf(dispbuf,"AT+CMGS=\"%s\"\r\n",phonenumber);
	Usart_SendString2(USART2,dispbuf);                             //Ӣ�Ķ���ָ��3
	delay_ms(200);
	while(Find_char((char*)Usart2_buff,"OK")); 
	printf("English_message_OK5\r\n");
	
	Usart_SendString2(USART2,message);                              //Ӣ�Ķ���ָ��4
	delay_ms(200);
	while(Find_char((char*)Usart2_buff,"OK"));  
	printf("English_message_OK6\r\n");
	
	Usart_SendHalfWord(USART2,0x1a);                                //����ָ��
	delay_ms(2000);  //��ʱ����
	while(Find_char((char*)Usart2_buff,"OK"));  
	printf("English_message_OK7\r\n");
} 

/**************************************************************************/
//�������ã��������Ķ��ź���
//�������ƣ�sim900a_send_Chinese_message(char *message,char *phonenumber)(uint8_t number);
//�ڲ�������message phonenumber
//�޸����ڣ�2022��1��26��  �賿2��40
//���ߣ�       (CSDN��)��ƨ��
/**************************************************************************/
void sim900a_send_Chinese_message(char *message,char *phonenumber)
{ 
	
	Usart_SendString2(USART2,"AT\r\n");                           //SIM900A�Ƿ��뵥Ƭ�������ӳɹ�
	delay_ms(200);	
	while(Find_char((char*)Usart2_buff,"OK"));                     //�ַ���ƥ�Ժ���   
	printf("Chinese_message_OK1\r\n");
	
	Usart_SendString2(USART2,"AT&F\r\n");                          //SIM900A��λ
	delay_ms(200);	
	while(Find_char((char*)Usart2_buff,"OK"));                    
	printf("Chinese_message_OK2\r\n");	
	
	Usart_SendString2(USART2,"AT+CSCS=\"UCS2\"\r\n");               //���Ķ���ָ��1
	delay_ms(200);	
	while(Find_char((char*)Usart2_buff,"OK"));                       
	printf("Chinese_message_OK3\r\n");
	
	
	Usart_SendString2(USART2,"AT+CMGF=1\r\n");                     //���Ķ���ָ��2
	delay_ms(200);	
	while(Find_char((char*)Usart2_buff,"OK"));  
	printf("Chinese_message_OK4\r\n");
	
	Usart_SendString2(USART2,"AT+CSMP=17,167,0,8\r\n");            //���Ķ���ָ��2
	delay_ms(200);	
	while(Find_char((char*)Usart2_buff,"OK"));  
	printf("Chinese_message_OK5\r\n");
	
	sprintf(dispbuf,"AT+CMGS=\"%s\"\r\n",phonenumber);
	Usart_SendString2(USART2,dispbuf);                             //���Ķ���ָ��3
	delay_ms(200);
	while(Find_char((char*)Usart2_buff,"OK")); 
	printf("Chinese_message_OK6\r\n");
	
	Usart_SendString2(USART2,message);                              //���Ķ���ָ��4
	delay_ms(200);
	while(Find_char((char*)Usart2_buff,"OK"));  
	printf("Chinese_message_OK7\r\n");
	
	Usart_SendHalfWord(USART2,0x1a);                                //���Ľ���ָ��
	delay_ms(2000);  //��ʱ����
	while(Find_char((char*)Usart2_buff,"OK"));  
	printf("Chinese_message_OK8\r\n");
} 

extern float longitude, latitude, altitude;		//���ȡ�ά�ȡ�����

//sim900A��������
void sim900a_send(void)
{
	//���Ͷ���
	char English_message[100];
	sprintf(English_message, "Axis is %.5fE %.5fN  %.1fM on fire", longitude, latitude, altitude);		//Ӣ�Ķ�������
	char phonenumber[]={"17328967102"};                             //���ܶ��ŵĺ���
	
	Usart2_Init(115200);		//��ʼ������2
	
	printf("Usart2_Init\r\n");
	
	//���Ե���Ʊ�ʾһ�·���
	LED0 = 0;
	delay_ms(1000);
	delay_ms(1000);
	delay_ms(1000);
	sim900a_send_English_message(English_message, phonenumber);
	LED0 = 1;

}





