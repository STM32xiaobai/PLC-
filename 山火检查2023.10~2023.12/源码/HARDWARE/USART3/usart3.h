#ifndef __USART3_H
#define __USART3_H	 
#include "sys.h"  
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32������
//����3��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2014/3/29
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	   

#define USART3_MAX_RECV_LEN		600					//�����ջ����ֽ���
#define USART3_MAX_SEND_LEN		600					//����ͻ����ֽ���
#define USART3_RX_EN 			1					//0,������;1,����.

extern u8  USART3_RX_BUF[USART3_MAX_RECV_LEN]; 		//���ջ���,���USART3_MAX_RECV_LEN�ֽ�
extern u8  USART3_TX_BUF[USART3_MAX_SEND_LEN]; 		//���ͻ���,���USART3_MAX_SEND_LEN�ֽ�
extern vu16 USART3_RX_STA;   						//��������״̬

void usart3_init(u32 bound);				//����2��ʼ�� 
void u3_printf(char* fmt,...);


void Usart2_Init(unsigned int baud);
void Usart_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch);
void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch);
void Usart_SendString(USART_TypeDef *USARTx, unsigned char *str, unsigned short len);
void Usart_SendString2( USART_TypeDef * pUSARTx, char *str);
void UsartPrintf(USART_TypeDef *USARTx, char *fmt,...);
#endif













