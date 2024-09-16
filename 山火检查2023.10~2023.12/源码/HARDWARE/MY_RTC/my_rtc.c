#include "my_rtc.h"
#include "sys.h"

#include "stm32f10x.h"                  // Device header
#include <time.h>

uint16_t MyRTC_Time[] = {2023, 1, 1, 23, 59, 55};	//����ȫ�ֵ�ʱ�����飬�������ݷֱ�Ϊ�ꡢ�¡��ա�ʱ���֡���

void MyRTC_SetTime(void);				//��������


/**
  * ��    ����RTC��ʼ��
  * ��    ������
  * �� �� ֵ����
  */
void MyRTC_Init(void)
{
	/*����ʱ��*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);		//����PWR��ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_BKP, ENABLE);		//����BKP��ʱ��
	
	/*���ݼĴ�������ʹ��*/
	PWR_BackupAccessCmd(ENABLE);							//ʹ��PWR�����Ա��ݼĴ����ķ���
	
	if (BKP_ReadBackupRegister(BKP_DR1) != 0xA5A5)			//ͨ��д�뱸�ݼĴ����ı�־λ���ж�RTC�Ƿ��ǵ�һ������
															//if������ִ�е�һ�ε�RTC����
	{
		RCC_LSEConfig(RCC_LSE_ON);							//����LSEʱ��
		while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) != SET);	//�ȴ�LSE׼������
		
		RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);				//ѡ��RTCCLK��ԴΪLSE
		RCC_RTCCLKCmd(ENABLE);								//RTCCLKʹ��
		
		RTC_WaitForSynchro();								//�ȴ�ͬ��
		RTC_WaitForLastTask();								//�ȴ���һ�β������
		
		RTC_SetPrescaler(32768 - 1);						//����RTCԤ��Ƶ����Ԥ��Ƶ��ļ���Ƶ��Ϊ1Hz
		RTC_WaitForLastTask();								//�ȴ���һ�β������
		
		MyRTC_SetTime();									//����ʱ�䣬���ô˺�����ȫ��������ʱ��ֵˢ�µ�RTCӲ����·
		
		BKP_WriteBackupRegister(BKP_DR1, 0xA5A5);			//�ڱ��ݼĴ���д���Լ��涨�ı�־λ�������ж�RTC�ǲ��ǵ�һ��ִ������
	}
	else													//RTC���ǵ�һ������
	{
		RTC_WaitForSynchro();								//�ȴ�ͬ��
		RTC_WaitForLastTask();								//�ȴ���һ�β������
	}
}


/**
  * ��    ����RTC����ʱ��
  * ��    ������
  * �� �� ֵ����
  * ˵    �������ô˺�����ȫ��������ʱ��ֵ��ˢ�µ�RTCӲ����·
  */
void MyRTC_SetTime(void)
{
	time_t time_cnt;		//�������������������
	struct tm time_date;	//��������ʱ����������
	
	time_date.tm_year = MyRTC_Time[0] - 1900;		//�������ʱ�丳ֵ������ʱ��ṹ��
	time_date.tm_mon = MyRTC_Time[1] - 1;
	time_date.tm_mday = MyRTC_Time[2];
	time_date.tm_hour = MyRTC_Time[3];
	time_date.tm_min = MyRTC_Time[4];
	time_date.tm_sec = MyRTC_Time[5];
	
	time_cnt = mktime(&time_date) - 8 * 60 * 60;	//����mktime������������ʱ��ת��Ϊ���������ʽ
													//- 8 * 60 * 60Ϊ��������ʱ������
	
	RTC_SetCounter(time_cnt);						//���������д�뵽RTC��CNT��
	RTC_WaitForLastTask();							//�ȴ���һ�β������
}

/**
  * ��    ����RTC��ȡʱ��
  * ��    ������
  * �� �� ֵ����
  * ˵    �������ô˺�����RTCӲ����·��ʱ��ֵ��ˢ�µ�ȫ������
  */
void MyRTC_ReadTime(void)
{
	time_t time_cnt;		//�������������������
	struct tm time_date;	//��������ʱ����������
	
	time_cnt = RTC_GetCounter() + 8 * 60 * 60;		//��ȡRTC��CNT����ȡ��ǰ���������
													//+ 8 * 60 * 60Ϊ��������ʱ������
	
	time_date = *localtime(&time_cnt);				//ʹ��localtime���������������ת��Ϊ����ʱ���ʽ
	
	MyRTC_Time[0] = time_date.tm_year + 1900;		//������ʱ��ṹ�帳ֵ�������ʱ��
	MyRTC_Time[1] = time_date.tm_mon + 1;
	MyRTC_Time[2] = time_date.tm_mday;
	MyRTC_Time[3] = time_date.tm_hour;
	MyRTC_Time[4] = time_date.tm_min;
	MyRTC_Time[5] = time_date.tm_sec;
}
