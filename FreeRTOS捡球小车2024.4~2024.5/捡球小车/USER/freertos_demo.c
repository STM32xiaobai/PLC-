#include "freertos_demo.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "malloc.h"
#include <stdlib.h>

//freertos����
#include "FreeRTOS.h"
#include "event_groups.h"
#include "task.h"
#include "semphr.h"

extern uint16_t x_num, y_num, colo_high, colo_low;
extern uint8_t Right_Speed, Left_Speed;		//��ȡopenmv���͵�����
extern uint8_t Su03t_Data;

//��ʼ����
#define START_TASK_PRIO		1
//�����ջ��С	
#define START_TASK_STACK_SIZE 		128  
//������
TaskHandle_t start_task_handler;
//������
void start_task(void *pvParameters);

//task1
#define TASK1_TASK_PRIO		2
//�����ջ��С	
#define TASK1_STK_SIZE 		128  
//������
TaskHandle_t task1_handler;
//������
void task1(void *pvParameters);

//task2
#define TASK2_TASK_PRIO		3
#define TASK2_STK_SIZE 		128  
TaskHandle_t task2_handler;
void task2(void *pvParameters);

//task3
#define TASK3_TASK_PRIO		4
#define TASK3_STK_SIZE 		128  
TaskHandle_t task3_handler;
void task3(void *pvParameters);

void timer_50ms_callback(TimerHandle_t pxTimer);
void timer_100ms_callback(TimerHandle_t pxTimer);

//*************************�¼���־λ****************************************
EventGroupHandle_t eventgroup_handle;	//�¼���־λ���
#define CONTROL_MOTOR (1 << 0)		//�����������
#define CONTROL_SG90	(1 << 1)		//����SG90���
#define OLED_SHOW			(1 << 2)		//OLED��ʾ
#define USART_SU03T		(1 << 3)		//��������ģ�������
#define EVEVT_ALL			(CONTROL_MOTOR | CONTROL_SG90 | OLED_SHOW | USART_SU03T)
//******************************************************************************

void freertos_demo(void)
{    
    xTaskCreate((TaskFunction_t         )   start_task,
                (char *                 )   "start_task",
                (uint16_t								)   START_TASK_STACK_SIZE,	//��ջ��С
                (void *                 )   NULL,
                (UBaseType_t            )   START_TASK_PRIO,			//���ȼ�
                (TaskHandle_t *         )   &start_task_handler );
    vTaskStartScheduler();
}


TimerHandle_t timer_50ms_handle = 0;	//OLEDˢ�¶�ʱ��
TimerHandle_t timer_100ms_handle = 0;	//�����ڷ��͵����ݵ������
QueueHandle_t semphore_handle;			

//��ʼ����������
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //�����ٽ���
	
		semphore_handle = xSemaphoreCreateBinary();		//������ֵ�ź���
		if(semphore_handle != NULL){ printf("��ֵ�ź��������ɹ�!!!\r\n"); }
	
		//�������ڶ�ʱ����OLED��ʾ��
		timer_50ms_handle = xTimerCreate( "50ms_timer", 
																	50,				//��ʱʱ��
																	pdTRUE,
																	(void *)1,
																	timer_50ms_callback );
																	
		//�������ڶ�ʱ����������յ������ݣ�
		timer_100ms_handle = xTimerCreate( "100ms_timer", 
																	100,				//��ʱʱ��
																	pdTRUE,
																	(void *)2,
																	timer_100ms_callback );
																	
		//�����¼���־λ														
		eventgroup_handle = xEventGroupCreate();
		if(eventgroup_handle != NULL){ printf("�¼���־λ�����ɹ���\r\n"); }
	
    //����TASK1����
    xTaskCreate((TaskFunction_t )task1,             
                (const char*    )"task1",           
                (uint16_t       )TASK1_STK_SIZE,        
                (void*          )NULL,                  
                (UBaseType_t    )TASK1_TASK_PRIO,        
                (TaskHandle_t*  )&task1_handler);   
    //����TASK2����
    xTaskCreate((TaskFunction_t )task2,     
                (const char*    )"task2",   
                (uint16_t       )TASK2_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )TASK2_TASK_PRIO,
                (TaskHandle_t*  )&task2_handler); 
		//����TASK3����
    xTaskCreate((TaskFunction_t )task3,     
                (const char*    )"task3",   
                (uint16_t       )TASK3_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )TASK3_TASK_PRIO,
                (TaskHandle_t*  )&task3_handler); 
								
    vTaskDelete(NULL); //ɾ����ʼ����
    taskEXIT_CRITICAL();            //�˳��ٽ���
}

//����1�������ƹ���
void task1(void *pvParameters)
{
	//���������ʱ��
	xTimerStart(timer_50ms_handle, portMAX_DELAY);
	xTimerStart(timer_100ms_handle, portMAX_DELAY);
	
	EventBits_t event_bit = 0;
	
	while(1)
	{
		event_bit = xEventGroupWaitBits(eventgroup_handle,			/* �¼���־���� */
																			EVEVT_ALL,						 /* �ȴ��¼���־���bit0��bit1λ */
																			pdTRUE,								/* �ɹ��ȴ����¼���־λ������¼���־���е�bit0��bit1λ */
																			pdFALSE,							 /* pdFALSE �� ; pdTRUE �� �����б�־λ�������*/
																			0);	

//		printf("�ȴ������¼���־λֵΪ��%#x\r\n",event_bit);
		
		//��������Ҫ��
		if(event_bit & CONTROL_MOTOR)
		{
			float distance = Get_Distance();		//���������
			
			if(distance > 10)	
			{	
				if(colo_low < 28)		//ɫ��С��Ҳ���Ǿ���Զ
				{
					if(x_num>1 && x_num<75)		//���ڳ�����ߣ�С����ת
					{
						Right_Motor(50);
//						Right_Motor(abs(95-x_num));
					}
					else if(x_num>115 && x_num<190)	//���ڳ����ұߣ�С����ת
					{
						Left_Motor(50);
//						Left_Motor(abs(95-x_num));
					}
					else if(x_num>=75 && x_num<=115)
					{
						Car_Run(50, 50);		//ֱ��
					}
					else
					{
						Car_Run(0, 0);
					}
				}
				else if(colo_low > 50)	//����̫������
				{
					Car_Run(-50, -50);
				}
				else if(x_num==0 && y_num==0 && colo_high==0 && colo_low==0)		//û��Ŀ��С����ת
				{
					Car_Turn();
				}	
			}
			else if(distance <= 3)		//ײǽ�˵���
			{
				Left_Motor(-50);
				Right_Motor(-50);
			}
//			printf("����1��������\r\n");
			xSemaphoreGive(semphore_handle);		//�ͷŶ�ֵ�ź���
		}	
//		printf("����1��������\r\n");
		vTaskDelay(10);
	}
}


//����2��OLED��ʾ����
void task2(void *pvParameters)
{
	BaseType_t err;
	while(1)
	{
		//�ö�ֵ�źſ�������OLED��ʾ
		err = xSemaphoreTake(semphore_handle, portMAX_DELAY);		//��ȡ��ֵ�ź���
		if(err == pdTRUE)
		{
			printf("��ֵ�ź�����ȡ�ɹ�!!!\r\n");
			OLED_Show();
		}
		else
		{
			printf("�ѳ�ʱ!!\r\n");
		}	
		printf("����2��������\r\n");
		vTaskDelay(10);
	}
}

//����ͻָ�����1��2
void task3(void *pvParameters)
{
	uint8_t key = 0;
	while(1)
	{
		key =  KEY_Scan(0);
		
		if(Su03t_Data == 1 || key == KEY0_PRES)
		{
			//����task1
			printf("����task1!!!\r\n");
			vTaskSuspend(task1_handler);
			vTaskSuspend(task2_handler);
		}
		if(Su03t_Data == 2 || key == KEY1_PRES)
		{
			//�ָ�task1
			printf("�ָ�task1!!!\r\n");
			vTaskResume(task1_handler);
			vTaskResume(task2_handler);
		}
		vTaskDelay(10);
	}
}

//**********************************************************************************************************
//***********************************************************************************************************

//�������ڶ�ʱ����OLED��ʾ��
void timer_50ms_callback(TimerHandle_t pxTimer)
{	
//	printf("��ʱ��1����\r\n");
}
//�������ڶ�ʱ����������յ������ݣ�
void timer_100ms_callback(TimerHandle_t pxTimer)
{
	xEventGroupSetBits(eventgroup_handle, CONTROL_MOTOR);
//	printf("��ʱ��2����\r\n");
}

//��ȡС�򲢷ŵ�����
void Clamp_Ball(void)
{
	Clamp_Close();
	vTaskDelay(1000);
	Elevator_Up();
	vTaskDelay(1000);
	Clamp_Open();
}

//С�������ջؼ���
void Put_Down_Clamp(void)
{
	Elevator_Down();
}

//OLED��ʾ����
void OLED_Show(void)
{
	
}







