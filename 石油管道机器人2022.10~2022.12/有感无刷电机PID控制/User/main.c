#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "./BSP/TIMER/bldc_tim.h"
#include "./BSP/KEY/key.h"
#include "./BSP/BLDC/bldc.h"
#include "pid.h"
#include "./DEBUG/debug.h"
#include "./BSP/ADC/adc.h"


extern float*user_setpoint;
extern float debug_data_temp;
extern int32_t motor_pwm_s;
extern int32_t temp_pwm1;
extern int16_t adc_amp_un[3];
extern float  adc_amp_bus;
extern uint16_t g_adc_value[ADC_CH_NUM * ADC_COLL];
extern uint8_t clc;
void bldc_speed_stop(void);                  /* ������״̬���رյ�� */

int main(void)
{
    uint8_t debug_cmd = 0;                   /* �����λ��ָ�� */
    float current_lpf[4] = {0.0f};           /* �����������Լ�ĸ�ߵ��� */
    uint8_t key,t;
    char buf[32];
    float current[3] = {0.0f};
    int16_t speed_diplay = 0;
    float user_setpoint_temp  = 0.0;

    HAL_Init();                              /* ��ʼ��HAL�� */
		sys_stm32_clock_init(RCC_PLL_MUL9);         /* ����ʱ��, 72Mhz */
    delay_init(168);                         /* ��ʱ��ʼ�� */
    usart_init(115200);                      /* ���ڳ�ʼ��Ϊ115200 */
    led_init();                              /* ��ʼ��LED */
    key_init();                              /* ��ʼ������ */
    bldc_init(72000/18-1,0);
    bldc_ctrl(MOTOR_1,CCW,0);                /* ��ʼ��ˢ����ӿ�1�ٶ� */
    pid_init();
   
    adc_nch_dma_init();
		

    
#if DEBUG_ENABLE                                                /* ��������*/
    debug_init();                                               /* PID���Գ�ʼ��*/
    debug_send_motorcode(BLDC_MOTOR );                          /* ֱ����ˢ��� */
    debug_send_motorstate(IDLE_STATE);                          /* ������� */
    /* ��ʼ��ͬ�����ݣ�ѡ���x��PIDX,Ŀ���ٶȵ�ַ,P,I,D����������λ�� */
    debug_send_initdata(TYPE_PID1,(float*)(&g_speed_pid.SetPoint),C_KP,C_KI,C_KD);    /* ������PID������PID1��*/
    debug_send_initdata(TYPE_PID2,(float*)(&g_speed_pid.SetPoint),S_KP,S_KI,S_KD);    /* �ٶȻ�PID������PID2��*/
#endif

    while (1)
    {
        t++;
			if(t % 60 == 0)
        {
            printf("PWM_Duty:%.1f%%",(float)((g_bldc_motor1.pwm_duty/MAX_PWM_DUTY)*100));/* ��ʾ����PWMռ�ձ� */
  
            printf("SetSpeed:%4d   ",(int16_t)*user_setpoint);     /* ��ʾ�����ٶ� */
  
            printf("M1 Speed:%4d   ",(int16_t)g_bldc_motor1.speed); /* ��ʾ�����ٶ� */
            
            printf("M1 pos:%4d",g_bldc_motor1.pos);                /* ��ʾλ�ñ仯 */
            
            printf("Power:%.3fV ",g_adc_value[0]*ADC2VBUS);				/* ��ʾ��Դ��ѹ */
            
            printf("Temp:%.1fC ",get_temp(g_adc_value[1]));				/* ��ʾ�¶� */
           
            LED0_TOGGLE();                                              /* LED0(���) ��ת */

            current[0] = adc_amp_un[0] * ADC2CURT; /* U */
            current[1] = adc_amp_un[1] * ADC2CURT; /* V */
            current[2] = adc_amp_un[2] * ADC2CURT; /* W */

            /* һ�������˲� �˲�ϵ��0.1 ������ʾ */
            FirstOrderRC_LPF(current_lpf[0],current[0],0.1f);
            FirstOrderRC_LPF(current_lpf[1],current[1],0.1f);
            FirstOrderRC_LPF(current_lpf[2],current[2],0.1f);
            FirstOrderRC_LPF(current_lpf[3],adc_amp_bus,0.1f);
            if(g_bldc_motor1.run_flag == STOP)                          /* ͣ���ĵ�����ʾ */
            {
                current_lpf[0]=0;
                current_lpf[1]=0;
                current_lpf[2]=0;
                current_lpf[3]=0;
            }
           
        }
				
				
				
        key = key_scan(0);
        if(key == KEY0_PRES)                                    /* ����KEY0Ŀ���ٶ�ֵ++ */
        {
            g_bldc_motor1.run_flag = RUN;                       /* �������� */
            start_motor1();                                     /* �������� */

            if(g_bldc_motor1.dir == CCW && *user_setpoint == 0) /* �л���������*/
            {
                g_bldc_motor1.dir = CW; 
            }
            *user_setpoint += 400;                              /* ��ʱ����ת�µ��� */
            if(*user_setpoint >= 3000)                          /* ��߲�����3000PRM */
                *user_setpoint = 3000;
            if(*user_setpoint == 0)
            {
                pid_init();                                     /* ��ʼ��PID */
                g_bldc_motor1.run_flag = STOP;                  /* ���ͣ�� */
                stop_motor1();                                  /* ͣ�� */
                g_bldc_motor1.speed = 0;
                motor_pwm_s = 0;
                g_bldc_motor1.pwm_duty = 0;
            }
            debug_data_temp = *user_setpoint;
        }
        else if(key == KEY1_PRES)                               /* ����KEY1Ŀ���ٶ�ֵ-- */
        {
            g_bldc_motor1.run_flag = RUN;                       /* �������� */
            start_motor1();                                     /* ���е�� */
            /* �л��������� */
            if(g_bldc_motor1.dir == CW && *user_setpoint == 0)
            {
                g_bldc_motor1.dir = CCW;
            }
            *user_setpoint -= 400;                              /* ��ʱ����ת�µ��� */
            if(*user_setpoint <= -3000)                         /* ��߲�����300PRM */
                *user_setpoint = -3000;
            if(*user_setpoint == 0)
            {
                pid_init();                                     /* ��ʼ��PID */
                g_bldc_motor1.run_flag = STOP;                  /* ���ͣ�� */
                stop_motor1();                                  /* ͣ�� */
                g_bldc_motor1.speed = 0;
                motor_pwm_s = 0;
                g_bldc_motor1.pwm_duty = 0;
            }

            debug_data_temp = *user_setpoint;
        }
        else if(key == KEY2_PRES)                               /* ����KEY2�رյ�� */
        {
            bldc_speed_stop();
        }
        
#if DEBUG_ENABLE
        /* Debug���Ͳ��� */
        /* ��Ҫ��ʾ���� */
        debug_send_valtage(g_adc_value[0] * ADC2VBUS);              /* ���͵�ѹ */
        debug_send_speed(g_bldc_motor1.speed);                      /* �����ٶ� */
        debug_send_temp(50,get_temp(g_adc_value[1]));               /* ���͵���¶ȡ��������¶� */
        debug_send_current((float)(current_lpf[0]/1000),(float)(current_lpf[0]/1000),(float)(current_lpf[0]/1000)); /* ���͵��� */
        /* �������κ��ٶȲ��� */
        debug_send_wave_data(1,(int16_t)g_bldc_motor1.speed);           /* ѡ��ͨ��1 ����ʵ���ٶ� */
        debug_send_wave_data(2,(int16_t)*user_setpoint);                /* ѡ��ͨ��2 ����Ŀ���ٶ� */ 
        debug_send_wave_data(3,current_lpf[0]);                         /* ѡ��ͨ��3 ����ʵ�ʵ���U */
        debug_send_wave_data(4,current_lpf[1]);                         /* ѡ��ͨ��4 ����ʵ�ʵ���V */
        debug_send_wave_data(5,current_lpf[2]);                         /* ѡ��ͨ��5 ����ʵ�ʵ���W */
        debug_send_wave_data(7,current_lpf[3]);                         /* ѡ��ͨ��7 ����ʵ�ʵ��� */
        debug_send_wave_data(8,g_current_pid.SetPoint);                 /* ѡ��ͨ��8 ����Ŀ����� */
        /* Debug���ղ��� */
       debug_receive_pid(TYPE_PID1,(float*)&g_current_pid.Proportion,  /* ��ѯ����PID���ֵ�PID1����*/
                          (float*)&g_current_pid.Integral,(float*)&g_current_pid.Derivative);
			 
       debug_cmd = debug_receive_ctrl_code();                      /* ��ȡ���� */
				
				
        if(debug_cmd == HALT_CODE)                                  /* ͣ�� */
        {
            pid_init();                                             /* ���³�ʼ��PID����ֹ���ֹ���ʧ�� */
            g_bldc_motor1.run_flag = STOP;                          /* ���ͣ�� */
            stop_motor1();                                          /* ͣ�� */
            g_bldc_motor1.speed = 0;
            motor_pwm_s = 0;
            g_bldc_motor1.pwm_duty = 0;
        }
        else if(debug_cmd == RUN_CODE)                              /* ���� */
        {
            g_bldc_motor1.dir = CW;
            g_bldc_motor1.run_flag = RUN;                           /* ���б�� */
            *user_setpoint = 400;                                   /* �Զ�����Ŀ�� */
            start_motor1();                                         /* ������� */
            debug_send_motorstate(RUN_STATE);                       /* ������� */
        }
        else if (debug_cmd == BREAKED)                              /* ɲ�������ֹͣ ���������вſɽ���� */
        {
            *user_setpoint = 0;                                     /* ����ֱ��0 */
            debug_send_motorstate(BREAKED_STATE);                   /* ���ɲ�� */
        }
#endif
       delay_ms(10); 
    }
}
/**
 * @brief       ������״̬���رյ��
 * @param       ��
 * @retval      ��
 */
void bldc_speed_stop(void)
{
    pid_init();                     /* ���³�ʼ��PID����ֹ���ֹ���ʧ�� */
    g_bldc_motor1.run_flag = STOP;  /* ���ͣ�� */
    stop_motor1();                  /* ͣ�� */
    g_bldc_motor1.speed = 0;
    motor_pwm_s = 0;
    g_bldc_motor1.pwm_duty = 0;
}


