#include "sys.h"
#include "usart.h"	
#include "string.h"

uint16_t BlueRxBuffer[7]={0};	
uint8_t Right_Speed = 0, Left_Speed = 0;		//����
uint16_t x_num, y_num, colo_high, colo_low;

//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"					//FreeRTOSʹ��
#include "task.h"
#include "semphr.h"	  
#endif

//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 
 
#if EN_USART1_RX   //���ʹ���˽���
//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA=0;       //����״̬���	  
  
void uart_init(u32 bound){
	//GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��USART1��GPIOAʱ��
  
	//USART1_TX   GPIOA.9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.9
   
	//USART1_RX	  GPIOA.10��ʼ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.10  

	//Usart1 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=7 ;//��ռ���ȼ�7
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
  
	//USART ��ʼ������

	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

	USART_Init(USART1, &USART_InitStructure); //��ʼ������1
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
	USART_Cmd(USART1, ENABLE);                    //ʹ�ܴ���1 
}


void USART1_IRQHandler(void)                	//����1�жϷ������
{
	u8 Res;
	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
		Res =USART_ReceiveData(USART1);	//��ȡ���յ�������
		
		if((USART_RX_STA&0x8000)==0)//����δ���
			{
			if(USART_RX_STA&0x4000)//���յ���0x0d
				{
				if(Res!=0x0a)USART_RX_STA=0;//���մ���,���¿�ʼ
				else USART_RX_STA|=0x8000;	//��������� 
				}
			else //��û�յ�0X0D
				{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
					{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
					}		 
				}
			}   		 
     } 
} 
#endif	





//************************************************��ӵĴ���***************************************************************************
//��ʼ��USART2����openmv������(����2��3��APB1)
void USART2_Init(u32 bound)
{
	//GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  
	//USART2_TX   GPIOA2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);
   
	//USART2_RX	  GPIOA3
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure); 

	//Usart1 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=7 ;//��ռ���ȼ�7
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
  
	//USART ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

	USART_Init(USART2, &USART_InitStructure); //��ʼ������1
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
	USART_Cmd(USART2, ENABLE);                    //ʹ�ܴ���1 
}

void USART2_IRQHandler(void)      
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) 
	{
		USART_ClearITPendingBit(USART2,USART_IT_RXNE); 
		uint16_t com_data = USART_ReceiveData(USART2);
		
		Openmv_Receive(com_data);
	}
}


//����openmv������
void Openmv_Receive(uint16_t com_data)
{
	uint8_t i;
	static uint8_t BlueRxCounter=0;//����
	static uint8_t BlueRxState = 0, OpenmvRxState = 0;	//BlueRxState�����ܵ�������OpenmvRxState��δ���ܵ�����ԭ����ת

//���յ���С��λ��
	if(BlueRxState==0&&com_data==0x6A)  //0x6A֡ͷ
	{			
		BlueRxState = 1;
		BlueRxBuffer[BlueRxCounter++]=com_data;  
	}
	else if(BlueRxState==1)
	{
		BlueRxBuffer[BlueRxCounter++] = com_data;
		if(BlueRxCounter>=7||com_data == 0x6D)		//0x6D֡β
		{
			BlueRxState = 2;
			//ȡ������
			x_num = BlueRxBuffer[BlueRxCounter-5];
			y_num = BlueRxBuffer[BlueRxCounter-4];
			colo_high = BlueRxBuffer[BlueRxCounter-3];
			colo_low = BlueRxBuffer[BlueRxCounter-2];
//			printf("%d, %d, %d, %d\r\n",x_num, y_num, colo_high, colo_low);
		}
	}
	else if(BlueRxState==2)		//����Ƿ���ܵ�������־
	{
			if(BlueRxBuffer[BlueRxCounter] == 0x6A)    
			{
						BlueRxState = 0;
						BlueRxCounter = 0;				
			}
			else   //���մ���
			{
						BlueRxState  = 0;
						BlueRxCounter =0;
						for(i=0;i<7;i++)
						{
								BlueRxBuffer[i]=0x00;      //�����������������
						}
			}
	} 
	
	
	//û�н��յ�С��λ��
	else if(BlueRxState==0&&com_data==0x6B)  //0x6B֡ͷ
	{
		OpenmvRxState = 1;
		BlueRxBuffer[BlueRxCounter++]=com_data;  
	}
	else if(OpenmvRxState==1)
	{
		BlueRxBuffer[BlueRxCounter++] = com_data;
		if(BlueRxCounter>=7||com_data == 0x6D)		//0x6D֡β
		{
			OpenmvRxState = 2;

			x_num = 0;
			y_num = 0;
			colo_high = 0;
			colo_low = 0;	
		}
	}
	else if(OpenmvRxState==2)		//����Ƿ���ܵ�������־
	{
			if(BlueRxBuffer[BlueRxCounter] == 0x6D)    
			{
						OpenmvRxState = 0;
						BlueRxCounter = 0;				
			}
			else   //���մ���
			{
						OpenmvRxState  = 0;
						BlueRxCounter =0;
						for(i=0;i<7;i++)
						{
								BlueRxBuffer[i]=0x00;      //�����������������
						}
			}
	} 
 
}


//su-03t����ģ�鴮�ڽ���
//��ʼ��USART3����openmv������(����2��3��APB1)
void USART3_Init(u32 bound)
{
	//GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
  
	//USART3_TX   GPIOB10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
	GPIO_Init(GPIOB, &GPIO_InitStructure);
   
	//USART3_RX	  GPIOB11
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	GPIO_Init(GPIOB, &GPIO_InitStructure); 

	//Usart1 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=7 ;//��ռ���ȼ�7
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
  
	//USART ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

	USART_Init(USART3, &USART_InitStructure); //��ʼ������1
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
	USART_Cmd(USART3, ENABLE);                    //ʹ�ܴ���1 
}

void USART3_IRQHandler(void)      
{
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) 
	{
		USART_ClearITPendingBit(USART3,USART_IT_RXNE); 
		uint16_t com_data = USART_ReceiveData(USART3);
		
		Su03t_Receive_Data(com_data);
	}
}


uint8_t Su03tRxBuffer[7] = {0};
uint8_t Su03t_Data = 0;
//su03t���պ���
void Su03t_Receive_Data(uint8_t com_data)
{
	uint8_t i;
	static uint8_t BlueRxCounter=0;//����
	static uint8_t BlueRxState = 0;	//����״̬

	if(BlueRxState==0&&com_data==0xAA)  //0x52֡ͷ
	{			
		BlueRxState = 1;
		Su03tRxBuffer[BlueRxCounter++]=com_data;  
	}
	else if(BlueRxState==1&&com_data==0x55)  //0x21֡ͷ
	{
		BlueRxState = 2;
		Su03tRxBuffer[BlueRxCounter++]=com_data;
	}	
	else if(BlueRxState==2)
	{	
		//����
		Su03tRxBuffer[BlueRxCounter++]=com_data;
		if(BlueRxCounter>=7||com_data == 0x55)    
		{
			BlueRxState = 3;       
			Su03t_Data =  Su03tRxBuffer[BlueRxCounter-2];
		}
	}

	else if(BlueRxState==3)		//����Ƿ���ܵ�������־
	{
			if(Su03tRxBuffer[BlueRxCounter-1] == 0xAA)    
			{
						BlueRxState = 0;
						BlueRxCounter = 0;				
			}
			else   //���մ���
			{
						BlueRxState  = 0;
						BlueRxCounter =0;
						for(i=0;i<7;i++)
						{
								Su03tRxBuffer[i]=0x00;      //�����������������
						}
			}
	} 
	else   //�����쳣
	{
			BlueRxState = 0;
			BlueRxCounter = 0;
			for(i=0;i<7;i++)
			{
					Su03tRxBuffer[i]=0x00;      //�����������������
			}
	}
}






























