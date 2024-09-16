#include "adc.h"
#include "delay.h"
#include "dma.h"
#include "bldc.h"
#include "bldc_tim.h"
#include "adc.h"

/* ��ͨ��ADC�ɼ� DMA��ȡ */
ADC_HandleTypeDef g_adc_nch_dma_handle;     /* ��DMA������ADC��� */
DMA_HandleTypeDef g_dma_nch_adc_handle;     /* ��ADC������DMA��� */
uint8_t g_adc_dma_sta = 0;                  /* DMA����״̬��־, 0,δ���; 1, ����� */

uint16_t g_adc_value[ADC_CH_NUM * ADC_COLL] = {0};     /* �洢ADCԭʼֵ */
float g_adc_u_value[ADC_CH_NUM] = {0};      /* �洢ADCת����ĵ�ѹֵ */

/***************************************��ͨ��ADC�ɼ�(DMA��ȡ)����*****************************************/
/**
 * @brief       ADC��ʼ��
 * @param       ��
 * @retval      ��
 */
void adc_init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    g_adc_nch_dma_handle.Instance = ADC1;
    g_adc_nch_dma_handle.Init.ScanConvMode = ADC_SCAN_ENABLE;                                /* ɨ��ģʽ ��ͨ��ʹ�� */
    g_adc_nch_dma_handle.Init.ContinuousConvMode = ENABLE;                          /* ����ת��ģʽ��ת�����֮����ż���ת�� */
    g_adc_nch_dma_handle.Init.DiscontinuousConvMode = DISABLE;                      /* ��ֹ����������ģʽ */
    g_adc_nch_dma_handle.Init.ExternalTrigConv = ADC_SOFTWARE_START;                /* ������� */
    g_adc_nch_dma_handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;                      /* �Ҷ��� */
    g_adc_nch_dma_handle.Init.NbrOfConversion = ADC_CH_NUM;                         /* ʹ��ת��ͨ�����������ʵ��ת��ͨ��ȥ���� */
    HAL_ADC_Init(&g_adc_nch_dma_handle);

    /* ����ʹ�õ�ADCͨ��������������ĵڼ���ת�������ӻ��߼���ͨ����Ҫ�޸��ⲿ�� */
    sConfig.Channel = ADC_CHANNEL_0;							/* ��Դ��ѹ�ɼ� */
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    HAL_ADC_ConfigChannel(&g_adc_nch_dma_handle, &sConfig);

    sConfig.Channel = ADC_CHANNEL_1;							/* �¶Ȳɼ� */
    sConfig.Rank = ADC_REGULAR_RANK_2;
    HAL_ADC_ConfigChannel(&g_adc_nch_dma_handle, &sConfig);

    sConfig.Channel = ADC_CHANNEL_2;							/* U������ɼ� */
    sConfig.Rank = ADC_REGULAR_RANK_3;
    HAL_ADC_ConfigChannel(&g_adc_nch_dma_handle, &sConfig);
    
    sConfig.Channel = ADC_CHANNEL_3;							/* V������ɼ� */
    sConfig.Rank = ADC_REGULAR_RANK_4;
    HAL_ADC_ConfigChannel(&g_adc_nch_dma_handle, &sConfig);
    
    sConfig.Channel = ADC_CHANNEL_4;							//W������ɼ�
    sConfig.Rank = ADC_REGULAR_RANK_5;
    HAL_ADC_ConfigChannel(&g_adc_nch_dma_handle, &sConfig);
    
}

/**
 * @brief       ADC DMA��ȡ ��ʼ������
 * @note        ����������ʹ��adc_init��ADC���д󲿷�����,�в���ĵط��ٵ�������
 * @param       par         : �����ַ
 * @param       mar         : �洢����ַ
 * @retval      ��
 */
void adc_nch_dma_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
      
    __HAL_RCC_ADC1_CLK_ENABLE();           /* ʹ��ADCxʱ�� */
    ADC_ADCX_CH0_GPIO_CLK_ENABLE();      /* ����GPIOʱ�� */
    ADC_ADCX_CH1_GPIO_CLK_ENABLE();
    ADC_ADCX_CH2_GPIO_CLK_ENABLE();
    ADC_ADCX_CH3_GPIO_CLK_ENABLE();
    ADC_ADCX_CH4_GPIO_CLK_ENABLE();
    
    /* AD�ɼ�����ģʽ����,ģ������ */
    GPIO_InitStruct.Pin = ADC_ADCX_CH0_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ADC_ADCX_CH0_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ADC_ADCX_CH1_GPIO_PIN;
    HAL_GPIO_Init(ADC_ADCX_CH1_GPIO_PORT, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = ADC_ADCX_CH2_GPIO_PIN;   
    HAL_GPIO_Init(ADC_ADCX_CH2_GPIO_PORT, &GPIO_InitStruct); 
    
    GPIO_InitStruct.Pin = ADC_ADCX_CH3_GPIO_PIN;   
    HAL_GPIO_Init(ADC_ADCX_CH3_GPIO_PORT, &GPIO_InitStruct); 
    
    GPIO_InitStruct.Pin = ADC_ADCX_CH4_GPIO_PIN;   
    HAL_GPIO_Init(ADC_ADCX_CH4_GPIO_PORT, &GPIO_InitStruct); 
    
    adc_init();
		
		__HAL_RCC_DMA1_CLK_ENABLE();                            /* DMA1ʱ��ʹ�� */

    /* DMA���� */
    g_dma_nch_adc_handle.Instance = DMA1_Channel1;                             /* ����DMAͨ�� */

    g_dma_nch_adc_handle.Init.Direction = DMA_PERIPH_TO_MEMORY;                 /* DIR = 1 ,  ���赽�洢��ģʽ */
    g_dma_nch_adc_handle.Init.PeriphInc = DMA_PINC_DISABLE;                     /* ���������ģʽ */
    g_dma_nch_adc_handle.Init.MemInc = DMA_MINC_ENABLE;                         /* �洢������ģʽ */
    g_dma_nch_adc_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;    /* �������ݳ���:16λ */
    g_dma_nch_adc_handle.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;       /* �洢�����ݳ���:16λ */
    g_dma_nch_adc_handle.Init.Mode = DMA_CIRCULAR;                              /* ��������ģʽ */
    g_dma_nch_adc_handle.Init.Priority = DMA_PRIORITY_MEDIUM;                   /* �е����ȼ� */
    HAL_DMA_Init(&g_dma_nch_adc_handle);
 
    __HAL_LINKDMA(&g_adc_nch_dma_handle,DMA_Handle,g_dma_nch_adc_handle);

		HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 2, 1);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    
    HAL_ADC_Start_DMA(&g_adc_nch_dma_handle,(uint32_t *)g_adc_value,ADC_CH_NUM * ADC_COLL);
 }

/**
 * @brief       ADC DMA�ɼ��жϷ�����
 * @param       �� 
 * @retval      ��
 */
void DMA1_Channel1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&g_dma_nch_adc_handle);
}

uint16_t g_adc_val[ADC_CH_NUM];           /*ADCƽ��ֵ�������*/

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1) //��Լ2.6ms�ɼ���ɽ����ж�
    { 
        HAL_ADC_Stop_DMA(&g_adc_nch_dma_handle);  /*�ر�DMAת��*/
        calc_adc_val(g_adc_val);      /*ADC��ֵת��*/
        HAL_ADC_Start_DMA(&g_adc_nch_dma_handle, (uint32_t *)&g_adc_value, (uint32_t)(ADC_SUM)); /*������DMAת��*/
    }
}


/**
 * @brief       ��ȡͨ��ch��ת��ֵ��ȡtimes��, Ȼ��ƽ��
 * @param       ch      : ͨ����, 0~17
 * @retval      ͨ��ch��times��ת�����ƽ��ֵ
 */
uint32_t adc_get_result_average(uint8_t ch)
{
    uint32_t temp_val = 0;
    uint16_t t;

    for (t = ch; t < ADC_SUM; t += ADC_CH_NUM )     /* ��ȡtimes������ */
    {
        temp_val += g_adc_value[t];
    }

    return temp_val / ADC_COLL;        /* ����ƽ��ֵ */
}
