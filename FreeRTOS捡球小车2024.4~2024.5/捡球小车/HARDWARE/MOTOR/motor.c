#include "motor.h"
#include "sys.h"


//���PWM��ʼ��TIM3_CH1��2�� CH3��4		��PA6��PA7��PB0��PB1��
void Motor_PWM_Init(u16 arr,u16 psc)  //Prescaler=0, Period=7200-1
{		 		
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);// 
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);  //ʹ��GPIO����ʱ��ʹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);  //ʹ��GPIO����ʱ��ʹ��
	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  ����Ƶ
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_Pulse = 0;                            //���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //�������:TIM����Ƚϼ��Ը�
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);  //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);  //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);  //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx

  TIM_CtrlPWMOutputs(TIM3,ENABLE);	//MOE �����ʹ��	

	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);  //CH4Ԥװ��ʹ��	 
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //CH4Ԥװ��ʹ��	
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);  //CH4Ԥװ��ʹ��	 
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);  //CH4Ԥװ��ʹ��	
	
	TIM_ARRPreloadConfig(TIM3, ENABLE); //ʹ��TIMx��ARR�ϵ�Ԥװ�ؼĴ���
	
	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIM1
} 


//����openmv�����ݿ��Ƴ����˶�
void Car_Run(int Left_Speed, int Right_Speed)
{
	Left_Motor(Left_Speed);
	Right_Motor(Right_Speed);
	
	Left_Speed = 0;
	Left_Speed = 0;
}


//û�н��յ�С�򣬳�����ת
void Car_Turn(void)
{
	Left_Motor(50);
	Right_Motor(-50);
}


//�����˶�
void Left_Motor(int PWM)
{
	//�޷�
	if(PWM > 80){ PWM = 80; }
	if(PWM < 80){ PWM = -80; }
	
	if(PWM > 0)
	{
		TIM_SetCompare1(TIM3, PWM);
		TIM_SetCompare2(TIM3, 0);
	}
	if(PWM <= 0)
	{
		TIM_SetCompare1(TIM3, 0);
		TIM_SetCompare2(TIM3, PWM);
	}
}

//�����˶�
void Right_Motor(int PWM)
{
	//�޷�
	if(PWM > 80){ PWM = 80; }
	if(PWM < 80){ PWM = -80; }
	
	if(PWM > 0)
	{
		TIM_SetCompare3(TIM3, PWM);
		TIM_SetCompare4(TIM3, 0);
	}
	if(PWM <= 0)
	{
		TIM_SetCompare4(TIM3, 0);
		TIM_SetCompare3(TIM3, PWM);
	}
}

//******************************************SG90�������*******************************************************
//TIM4_CH1��2��3          PB6��7��8
//ͨ��1����ȡƹ����ļ���			ͨ��2��3���������ӵĶ��
void SG90_Init(void)
{
		// �ṹ�嶨��
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	// ����ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	// ��ʼ��GPIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;   // �����������
	GPIO_Init(GPIOB,&GPIO_InitStructure);
//	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3,ENABLE);   // �ı�ָ���ܽŵ�ӳ��	
	
	// ��ʼ����ʱ������
	TIM_TimeBaseInitStructure.TIM_Period = 200;   // �Զ�װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler = 7200-1;   // ��Ƶϵ��
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;   // �������ϼ���ģʽ
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);	
	
	// ��ʼ��PWM����
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;   // �Ƚ����ģʽ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;   // �������
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;   // ���ʹ��
	TIM_OC1Init(TIM4,&TIM_OCInitStructure);   // ����Ƚ�ͨ��1��ʼ��
	
	TIM_OC1PreloadConfig(TIM4,TIM_OCPreload_Enable);   // ʹ��TIMx�� CCR1 �ϵ�Ԥװ�ؼĴ���
	TIM_ARRPreloadConfig(TIM4,ENABLE);   // ʹ��Ԥװ�ؼĴ���
	
	TIM_Cmd(TIM4,ENABLE);   

}

//���ӱպ�
void Clamp_Close(void)
{
	TIM_SetCompare1(TIM4,195);   // ��ת��0��
}
//���Ӵ�
void Clamp_Open(void)
{
	TIM_SetCompare1(TIM4,185);   // ��ת��90��
}

//�����������
void Elevator_Up(void)
{
	TIM_SetCompare2(TIM4,180);   // ��ת��135��
	TIM_SetCompare3(TIM4,180);   // ��ת��135��
}

//�����������
void Elevator_Down(void)
{
	TIM_SetCompare2(TIM4,195);   // ��ת��0��
	TIM_SetCompare3(TIM4,195);   // ��ת��0��
}




















