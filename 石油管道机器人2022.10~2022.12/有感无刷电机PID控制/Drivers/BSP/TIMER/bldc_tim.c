#include "bldc_tim.h"
#include "led.h"
#include "bldc.h"
#include "pid.h"
#include "delay.h"
#include "adc.h"

/******************************************************************************************/
/* ��ʱ�����þ�� ���� */

/* �߼���ʱ��PWM */
TIM_HandleTypeDef g_atimx_handle;           /* ��ʱ��x��� */
TIM_OC_InitTypeDef g_atimx_oc_chy_handle;   /* ��ʱ�������� */
extern _bldc_obj g_bldc_motor1;
extern PID_TypeDef  g_speed_pid;            /* �ٶ�PID�����ṹ��*/
/******************************************************************************************/

/**
 * @brief       �߼���ʱ��TIMX PWM�����ʼ������
 * @note
 *              �߼���ʱ����ʱ������APB2, ��PCLK2 = 168Mhz, ��������PPRE2����Ƶ, ���
 *              �߼���ʱ��ʱ�� = 168Mhz
 *              ��ʱ�����ʱ����㷽��: Tout = ((arr + 1) * (psc + 1)) / Ft us.
 *              Ft=��ʱ������Ƶ��,��λ:Mhz
 *
 * @param       arr: �Զ���װֵ
 * @param       psc: ʱ��Ԥ��Ƶ��
 * @retval      ��
 */
void atim_timx_oc_chy_init(uint16_t arr, uint16_t psc)
{
    __HAL_RCC_TIM1_CLK_ENABLE();                             /* TIMX ʱ��ʹ�� */


    g_atimx_handle.Instance = TIM1;                    /* ��ʱ��x */
    g_atimx_handle.Init.Prescaler = psc;                        /* ��ʱ����Ƶ */
    g_atimx_handle.Init.CounterMode = TIM_COUNTERMODE_UP;       /* ���ϼ���ģʽ */
    g_atimx_handle.Init.Period = arr;                           /* �Զ���װ��ֵ */
    g_atimx_handle.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;   /* ��Ƶ���� */
    g_atimx_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE; /*ʹ��TIMx_ARR���л���*/
    g_atimx_handle.Init.RepetitionCounter = 0;                  /* ��ʼʱ������*/
    HAL_TIM_PWM_Init(&g_atimx_handle);                          /* ��ʼ��PWM */

    g_atimx_oc_chy_handle.OCMode = TIM_OCMODE_PWM1;             /* ģʽѡ��PWM1 */
    g_atimx_oc_chy_handle.Pulse = 0;							//CCRX����ֵ
    g_atimx_oc_chy_handle.OCPolarity = TIM_OCPOLARITY_HIGH;     /* ����Ƚϼ���Ϊ�� */
    g_atimx_oc_chy_handle.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    g_atimx_oc_chy_handle.OCFastMode = TIM_OCFAST_DISABLE;
    g_atimx_oc_chy_handle.OCIdleState = TIM_OCIDLESTATE_RESET;
    g_atimx_oc_chy_handle.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&g_atimx_handle, &g_atimx_oc_chy_handle, TIM_CHANNEL_1); /* ����TIMxͨ��y */
    HAL_TIM_PWM_ConfigChannel(&g_atimx_handle, &g_atimx_oc_chy_handle, TIM_CHANNEL_2); /* ����TIMxͨ��y */
    HAL_TIM_PWM_ConfigChannel(&g_atimx_handle, &g_atimx_oc_chy_handle, TIM_CHANNEL_3); /* ����TIMxͨ��y */

    /* ������ʱ��ͨ��1���PWM */
    HAL_TIM_PWM_Start(&g_atimx_handle,TIM_CHANNEL_1);

    /* ������ʱ��ͨ��2���PWM */
    HAL_TIM_PWM_Start(&g_atimx_handle,TIM_CHANNEL_2);

    /* ������ʱ��ͨ��3���PWM */
    HAL_TIM_PWM_Start(&g_atimx_handle,TIM_CHANNEL_3);
	
	HAL_TIM_Base_Start_IT(&g_atimx_handle);  /* �����߼���ʱ��1 *///�����жϲ�����������
}




/**
 * @brief       ��ʱ���ײ�������ʱ��ʹ�ܣ���������
                �˺����ᱻHAL_TIM_PWM_Init()����
 * @param       htim:��ʱ�����
 * @retval      ��
 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == ATIM_TIMX_PWM)
    {
        GPIO_InitTypeDef gpio_init_struct;
        __HAL_RCC_TIM1_CLK_ENABLE();                             /* ��ʱ��ʱ��ʹ�� */
        
        /* ���ű۵�IOʱ��ʹ�� */
        __HAL_RCC_GPIOE_CLK_ENABLE();                     
        __HAL_RCC_GPIOE_CLK_ENABLE(); 
			  __HAL_RCC_GPIOE_CLK_ENABLE(); 		
        
        /* ���ű۵�IOʱ��ʹ�� */
        __HAL_RCC_GPIOE_CLK_ENABLE();                         
        __HAL_RCC_GPIOE_CLK_ENABLE();                    
        __HAL_RCC_GPIOE_CLK_ENABLE(); 

		//��ӳ��
		__HAL_RCC_AFIO_CLK_ENABLE();
		__HAL_AFIO_REMAP_TIM1_ENABLE();
		

        /* ���ű۵�IO��ʼ�� */
        gpio_init_struct.Pin = GPIO_PIN_8;
        gpio_init_struct.Pull = GPIO_NOPULL;
        gpio_init_struct.Speed = GPIO_SPEED_HIGH;
        gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;                /* �������ģʽ */
        HAL_GPIO_Init(GPIOE, &gpio_init_struct);

        gpio_init_struct.Pin = GPIO_PIN_10;
        HAL_GPIO_Init(GPIOE, &gpio_init_struct);

        gpio_init_struct.Pin = GPIO_PIN_12;
        HAL_GPIO_Init(GPIOE, &gpio_init_struct);


        /* ���űۼ���ʱ��IO��ʼ�� */
        gpio_init_struct.Pin = GPIO_PIN_9;          /* ͨ��y��GPIO�� */
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;                    /* ����������� */
        gpio_init_struct.Pull = GPIO_NOPULL;                        /* ���� */
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;              /* ���� */
//        gpio_init_struct.Alternate = ATIM_TIMX_PWM_CHY_GPIO_AF;     /* �˿ڸ��� */
        HAL_GPIO_Init(GPIOE, &gpio_init_struct);

        gpio_init_struct.Pin = GPIO_PIN_11;          /* ͨ��y��CPIO�� */
        HAL_GPIO_Init(GPIOE, &gpio_init_struct);

        gpio_init_struct.Pin = GPIO_PIN_13;          /* ͨ��y��CPIO�� */
        HAL_GPIO_Init(GPIOE, &gpio_init_struct);
        
        HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 2, 2);
        HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

    }
}


/**
 * @brief       ��ʱ���жϷ�����
 * @param       ��
 * @retval      ��
 */
void TIM1_UP_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&g_atimx_handle);
}



/******************************************������ʱ����ʼ��**********************************************************/
TIM_HandleTypeDef timx_handler;         /* ��ʱ��������� */


/**
 * @brief       ������ʱ��TIMX��ʱ�жϳ�ʼ������
 * @note
 *              ������ʱ����ʱ������APB1,��PPRE1 �� 2��Ƶ��ʱ��
 *              ������ʱ����ʱ��ΪAPB1ʱ�ӵ�2��, ��APB1Ϊ42M, ���Զ�ʱ��ʱ�� = 842Mhz
 *              ��ʱ�����ʱ����㷽��: Tout = ((arr + 1) * (psc + 1)) / Ft us.
 *              Ft=��ʱ������Ƶ��,��λ:Mhz
 *
 * @param       arr: �Զ���װֵ��
 * @param       psc: ʱ��Ԥ��Ƶ��
 * @retval      ��
 */
void btim_timx_int_init(uint16_t arr, uint16_t psc)
{
    timx_handler.Instance = TIM2;                      /* ͨ�ö�ʱ��X */
    timx_handler.Init.Prescaler = psc;                          /* ����Ԥ��Ƶ��  */
    timx_handler.Init.CounterMode = TIM_COUNTERMODE_UP;         /* ���ϼ����� */
    timx_handler.Init.Period = arr;                             /* �Զ�װ��ֵ */
    timx_handler.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;   /* ʱ�ӷ�Ƶ���� */
    HAL_TIM_Base_Init(&timx_handler);
    
		HAL_TIM_Base_Start_IT(&timx_handler);                      /* ʹ��ͨ�ö�ʱ��x�ͼ�������жϣ�TIM_IT_UPDATE */
    __HAL_TIM_CLEAR_IT(&timx_handler,TIM_IT_UPDATE);            /* ��������жϱ�־λ */
}

/**
 * @brief       ��ʱ���ײ�����������ʱ�ӣ������ж����ȼ�
                �˺����ᱻHAL_TIM_Base_Init()��������
 * @param       ��
 * @retval      ��
 */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        __HAL_RCC_TIM2_CLK_ENABLE();                     /* ʹ��TIMʱ�� */
        HAL_NVIC_SetPriority(TIM2_IRQn, 1, 3); /* ��ռ1�������ȼ�3����2 */
        HAL_NVIC_EnableIRQ(TIM2_IRQn);         /* ����ITM3�ж� */
    }
}

/**
 * @brief       ������ʱ��TIMX�жϷ�����
 * @param       ��
 * @retval      ��
 */
void BTIM_TIMX_INT_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&timx_handler);                  /* ��ʱ���ص����� */
}



/***********************************************��ʱ���жϻص�����***********************************************/
static uint8_t pid_s_count = 0;         /* �ٶȻ����б�־ */
static uint8_t pid_c_count = 0;         /* ���������б�־ */
static uint8_t cf_count = 0;            /* ��ʱ��ʱ���¼ */
int32_t temp_pwm1 = 0;                  /* ����ٶȻ���PID������ */
int32_t temp_pwm2 = 0;                  /* ��ŵ�������PID������ */

/* ��PID����ֵ����һ���˲����������±��� */
int32_t motor_pwm_s = 0;
int32_t motor_pwm_c = 0;
int32_t motor_pwm_sl= 0;

/* ͣ��״̬�µ����ɼ�ʹ�� */
#define ADC_AMP_OFFSET_TIMES 50    
uint16_t adc_amp_offset[3][ADC_AMP_OFFSET_TIMES+1];
uint8_t adc_amp_offset_p = 0;
int16_t adc_amp[3];
volatile uint16_t adc_val_m1[ADC_CH_NUM];
int16_t adc_amp_un[3];
float adc_amp_bus = 0.0f;

float debug_data_temp = 0.0f;
float *user_setpoint = (float*)(&g_speed_pid.SetPoint); /* Ŀ��ֵ��ֵ */
uint8_t clc = 0;                                        /* �ȴ�ʱ������������ʱ�� */
/**
 * @brief       ��ʱ���жϻص�
 * @param       ��
 * @retval      ��
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    uint8_t i;
    uint8_t bldc_dir=0;
    static uint8_t times_count=0;                       /* ��ʱ��ʱ���¼ */
    int16_t temp_speed=0;                               /* ��ʱ�ٶȴ洢 */
    if(htim->Instance == ATIM_TIMX_PWM)                 /* 55us */
    {
        if(g_bldc_motor1.run_flag == RUN)
        {
            /******************************* ����������� *******************************/
            for(i=0; i<3; i++)
            {
                adc_val_m1[i] = g_adc_val[i+2];         /* UVW����ADCͨ�� */
                adc_amp[i] = adc_val_m1[i] - adc_amp_offset[i][ADC_AMP_OFFSET_TIMES];

               if(adc_amp[i] >= 0)                      /* ȥ�����綯������ĸ��������� */
               adc_amp_un[i] = adc_amp[i];
            }
            /* ����ĸ�ߵ�����ĸ�ߵ���Ϊ���������п��ض����������֮�ͣ�*/
            if(g_bldc_motor1.step_sta == 0x05)
            {
                adc_amp_bus = (adc_amp_un[0] + adc_amp_un[1])*ADC2CURT; /* UV */
            }
            else if(g_bldc_motor1.step_sta == 0x01)
            {
                adc_amp_bus = (adc_amp_un[0] + adc_amp_un[2])*ADC2CURT; /* UW */
            }
            else if(g_bldc_motor1.step_sta == 0x03)
            {
                adc_amp_bus = (adc_amp_un[1]+ adc_amp_un[2])*ADC2CURT;  /* VW */
            }
            else if(g_bldc_motor1.step_sta == 0x02)
            {
                adc_amp_bus = (adc_amp_un[0]+ adc_amp_un[1])*ADC2CURT;  /* UV */
            }
            else if(g_bldc_motor1.step_sta == 0x06)
            {
                adc_amp_bus= (adc_amp_un[0]+ adc_amp_un[2])*ADC2CURT;   /* WU */
            }
            else if(g_bldc_motor1.step_sta == 0x04)
            {
                adc_amp_bus= (adc_amp_un[2]+ adc_amp_un[1])*ADC2CURT;   /* WV */
            }
            
            /******************************* �������� *******************************/
            if(g_bldc_motor1.dir == CW)
            {
                g_bldc_motor1.step_sta = hallsensor_get_state(MOTOR_1);
            }
            else
            {
                g_bldc_motor1.step_sta = 7 - hallsensor_get_state(MOTOR_1);
            }
            if((g_bldc_motor1.step_sta <= 6)&&(g_bldc_motor1.step_sta >= 1))     
            {
                pfunclist_m1[g_bldc_motor1.step_sta-1]();
            }
            else    /* ���������󡢽Ӵ��������Ͽ������ */
            {
                stop_motor1();
                g_bldc_motor1.run_flag = STOP;
            }
            /******************************* �ٶȼ��� *******************************/
            g_bldc_motor1.count_j++;                /* �����ٶ�ר�ü���ֵ */
            g_bldc_motor1.hall_sta_edge = uemf_edge(g_bldc_motor1.hall_single_sta);
            if(g_bldc_motor1.hall_sta_edge == 0)    /* ͳ�Ƶ��������źŵĸߵ�ƽʱ�� */
            {
                /* �����ٶ� */
                if(g_bldc_motor1.dir == CW)
                    temp_speed = (SPEED_COEFF/g_bldc_motor1.count_j);
                else
                    temp_speed = -(SPEED_COEFF/g_bldc_motor1.count_j);
                FirstOrderRC_LPF(g_bldc_motor1.speed,temp_speed,0.2379);
                g_bldc_motor1.no_single = 0;
                g_bldc_motor1.count_j = 0;
            }
            if(g_bldc_motor1.hall_sta_edge == 1)
            {
                g_bldc_motor1.no_single = 0;
                g_bldc_motor1.count_j = 0;
            }
            if(g_bldc_motor1.hall_sta_edge == 2)            /* ����ֵһֱ�������δ���� */
            {
                g_bldc_motor1.no_single++;                  /* ������ʱ���ۼ� ��ʱ���ж��ٶ�Ϊ0 */
                if(g_bldc_motor1.no_single > 15000)
                {
                    g_bldc_motor1.no_single = 0;
                    g_bldc_motor1.speed = 0;                /* ��ʱ���� �ж�Ϊֹͣ �ٶ�Ϊ0 */
                }
            }
            /******************************* λ�ü�¼�Լ���ת��� *******************************/
            if(g_bldc_motor1.step_last != g_bldc_motor1.step_sta)
            {
                g_bldc_motor1.hall_keep_t = 0;
                bldc_dir = check_hall_dir(&g_bldc_motor1);
                if(bldc_dir == CCW)
                {
                    g_bldc_motor1.pos -= 1;
                }
                else if(bldc_dir == CW)
                {
                    g_bldc_motor1.pos += 1;
                }
                g_bldc_motor1.step_last = g_bldc_motor1.step_sta;
            }
            else if(g_bldc_motor1.run_flag == RUN)          /* �����һ�������ʱ */
            {
                g_bldc_motor1.hall_keep_t++;                /* ����һ���������ֵ��ʱ�䣩 ��λ1/18k */
                if(g_bldc_motor1.hall_keep_t > 15000)       /* ��ת */
                {
                    g_bldc_motor1.hall_keep_t = 0;
#if LOCK_TAC
                    stop_motor1();
                    g_bldc_motor1.run_flag = STOP;;         /* ���ͣ�� */
                    g_bldc_motor1.pwm_duty = 0;
#endif
                    g_bldc_motor1.locked_rotor = 1;         /* ��Ƕ�ת */
                }
            }
        /******************************* PID���� *******************************/
        if(g_bldc_motor1.run_flag == RUN)                   /* ����PID�ջ����� */
        {
            pid_c_count++;
            pid_s_count++;

            if(pid_s_count > 2)
            {
/* ������λ������ ��PIDִ��֮ǰ�Ե�����ֵ����Ԥ���ж� */
#if DEBUG_ENABLE 
                debug_set_point_range(3000,-3000,6000);     /* ����Ŀ����ڷ�Χ(3000~-3000)������󲽽�ֵ������6000 RPM */
                debug_data_temp = *user_setpoint;
                if(*user_setpoint < 0)                      /* ��λ��ָ�����л���ת���� */
                {
                    /* Ϊ�򻯿����߼� ֻ�Ե��״̬��Ϊ�����ݣ�ͬ����*/
                    if(g_bldc_motor1.speed == 0)
                    {
                        g_bldc_motor1.dir = CCW;
                    }
                    else if(g_bldc_motor1.dir == CW && g_bldc_motor1.speed != 0)/* ��ǰ״̬���������л� */
                    {
                        *user_setpoint = 0;
                    }
                }
                else if(*user_setpoint > 0)
                {
                    if(g_bldc_motor1.speed == 0)
                    {
                        g_bldc_motor1.dir = CW;
                    }
                    else if(g_bldc_motor1.dir == CCW && g_bldc_motor1.speed != 0)/* ��ǰ״̬���������л� */
                    {
                        *user_setpoint = 0;
                    }
                }
#endif
         /******************************* PID���� *******************************/
                temp_pwm1 = increment_pid_ctrl(&g_speed_pid,g_bldc_motor1.speed);
                FirstOrderRC_LPF(motor_pwm_s,temp_pwm1,0.085);
                if(motor_pwm_s < 0)
                {
                    motor_pwm_sl = -motor_pwm_s;
                }
                else
                {
                    motor_pwm_sl = motor_pwm_s;
                }

                *user_setpoint = debug_data_temp;   /* ���±�����λ��ָ��Ҫ�� */
                pid_s_count = 0;
            }

            if(pid_c_count > 1)                     /* ������ */
            {
                /* ��������������趨�ĵ���ֵ������PID����ת������������ �ٶȻ��޷������ã�ת���޷����� */
                if(adc_amp_bus > (g_current_pid.SetPoint - 20))
                {
                    cf_count++;                     /* �˳������������Ӱ�� */
                    if(cf_count > 4)
                    {
                        cf_count = 0;
                        temp_pwm2 = increment_pid_ctrl(&g_current_pid,adc_amp_bus);
                        FirstOrderRC_LPF(motor_pwm_c,temp_pwm2,0.085);/* һ�������˲� �˲�ϵ��0.08 */
                    }
                }
                else
                {
                    cf_count = 0;
                    temp_pwm2 = increment_pid_ctrl(&g_current_pid,adc_amp_bus);
                    FirstOrderRC_LPF(motor_pwm_c,temp_pwm2,0.085);
                }
                pid_c_count = 0;
            }
            /* ���������ֵ�����ٶȻ������ʹ���ٶȻ����� */
            if(motor_pwm_c > motor_pwm_sl)
            {
                g_bldc_motor1.pwm_duty = motor_pwm_sl;
                if(motor_pwm_s < 0)                     /* ����ת���ֿ��� */
                    g_current_pid.Ui = -g_speed_pid.Ui;
                else
                    g_current_pid.Ui = g_speed_pid.Ui;
            }
            else  /* �ٶȻ����ֵ���ڵ����������ʹ�õ��������� */
            {
                g_bldc_motor1.pwm_duty = motor_pwm_c;
                if(motor_pwm_s < 0)
                    g_speed_pid.Ui = -g_current_pid.Ui;
                else
                    g_speed_pid.Ui = g_current_pid.Ui;
            }
        }
        }
    }
    if(htim->Instance == TIM2)
    {
        /******************************* �ɼ����ͣ��״̬�µ�ƫ�õ�ѹ *******************************/
        times_count++;
        if(g_bldc_motor1.run_flag == STOP)
        {
            uint8_t i;
            uint32_t avg[3] = {0,0,0};
            adc_amp_offset[0][adc_amp_offset_p] = g_adc_val[2];     /* �õ���δ��ʼ�˶�ʱ����Ļ�׼��ѹ */
            adc_amp_offset[1][adc_amp_offset_p] = g_adc_val[3];
            adc_amp_offset[2][adc_amp_offset_p] = g_adc_val[4];

            adc_amp_offset_p++;
            NUM_CLEAR(adc_amp_offset_p,ADC_AMP_OFFSET_TIMES);       /* ���ɼ�ADC_AMP_OFFSET_TIMES�Σ���������0��ʼ�����ɼ� */
            for(i = 0; i < ADC_AMP_OFFSET_TIMES; i++)               /* ���ɼ���ÿ��ͨ��ֵ�ۼ� */
            {
                avg[0] += adc_amp_offset[0][i];
                avg[1] += adc_amp_offset[1][i];
                avg[2] += adc_amp_offset[2][i];
            }
            for(i = 0; i < 3; i++)                                  /* ȡƽ��������˲� */
            {
                avg[i] /= ADC_AMP_OFFSET_TIMES;
                adc_amp_offset[i][ADC_AMP_OFFSET_TIMES] = avg[i];   /* �õ���δ��ʼ�˶�ʱ�Ļ�׼��ѹ */
            }
        }
        /******************************* ��ʱ�жϵ���Ƿ������� *******************************/
        if(times_count == SMAPLSE_PID_SPEED)
        {
#if (LOCK_TAC == 2)
            if(g_bldc_motor1.locked_rotor == 1)         /* ��ת����������һ���ٶȺ�ɽ���ջ����� */
            {
                clc++;
                if(clc > 50)                            /* �ӳ�2s���������� */
                {
#if DEBUG_ENABLE /*��������*/
                    debug_send_motorstate(RUN_STATE);   /* �������*/
#endif
                    clc = 0;
                    pid_init();
                    stop_motor1();
                    g_speed_pid.SetPoint = 400.0;       /* 400PRM */
                    g_bldc_motor1.pwm_duty = 500;       /* ���������ٶ� */
                    g_bldc_motor1.run_flag = RUN;       /* �������� */
                    start_motor1();                     /* ���е�� */
                    g_bldc_motor1.locked_rotor = 0;
                }
            }
#endif
            times_count = 0;
        }

    }
}




