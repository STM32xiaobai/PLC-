#include "sys.h"
#include "distance.h"
#include "delay.h"

//��ʼ��������ģ��TIM2_CH2(PA1)
void Distance_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);  //ʹ��GPIO����ʱ��ʹ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //����TIM2ʱ��	
	
	//Trig����		PA4
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA,GPIO_Pin_4); 	//��ʼ�õ͵�
	
	//ECHO����	PA1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	TIM_TimeBaseInitStructure.TIM_Period = 65535;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 71;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
}

//����������һ����������
void frist_maichong(void)
{
	GPIO_SetBits(GPIOA, GPIO_Pin_4);
	delay_ms(30);
	GPIO_ResetBits(GPIOA, GPIO_Pin_4);
}

float Get_Distance(void)
{
	while (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1) == 1);//�ȵ�ECHOΪ�͵�ƽ�������ص�ֵ�ɹ��󣬲�������������
	frist_maichong();
	while (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1) == 0);//�ȵ�ECHO���շ�������ʱ����ʱ��
  TIM_SetCounter(TIM2,0); //���������
	TIM_Cmd(TIM2, ENABLE);  //ʹ�ܶ�ʱ��2,��ʼ����
  while (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1) == 1);//�ȵ�ECHOΪ�͵�ƽ��ֹͣ��ʱ
	TIM_Cmd(TIM2, DISABLE);	//ʧ�ܶ�ʱ��2,��ֹ����	
	
	return 	(TIM_GetCounter(TIM2))/1000000.0*340/2 *100;//��ʱ���ʱ��λcm
}
