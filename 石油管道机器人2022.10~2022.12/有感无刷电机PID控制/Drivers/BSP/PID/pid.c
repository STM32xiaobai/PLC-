#include "./BSP/PID/pid.h"
#include "./BSP/BLDC/bldc.h"

PID_TypeDef  g_current_pid;        /* λ��PID�����ṹ�� */
PID_TypeDef  g_speed_pid;           /* �ٶ�PID�����ṹ�� */
/**
 * @brief       ��ʼ��PID
 * @param       ��
 * @retval      ��
 */
void pid_init(void)
{
    /* �趨����Ŀ��1500mA(������С����400mA����)�ϸߵ�ת�ٶ�Ӧ�ĵ����ϴ����ؽϴ󣬿���Ӧ�ϸߵ�ת�ٵ���*/
    /* ��ע�⡿�����õ�ת�ٶ�Ӧ�ĵ��������˵����趨ֵ��������PIDת�����������ڣ�ת�ٽ��޷��������� */
    g_current_pid.SetPoint = 1000.0;
    g_current_pid.ActualValue = 0.0;    /* �趨Ŀ��Desired Value */
    g_current_pid.LastError = 0.0;      /* Error[1] */
    g_current_pid.LastError = 0.0;      /* Error[-1] */
    g_current_pid.PrevError = 0.0;      /* Error[-2] */
    g_current_pid.Proportion = C_KP;    /* �������� Proportional Const */
    g_current_pid.Integral = C_KI;      /* ���ֳ��� Integral Const */
    g_current_pid.Derivative = C_KD;    /* ΢�ֳ��� Derivative Const */
    g_current_pid.IngMax = 9000;
    g_current_pid.IngMin = 600;
    g_current_pid.OutMin = 600;
    g_current_pid.OutMax = 9000;
    
    g_speed_pid.SetPoint = 0;           /* �趨Ŀ��Desired Value */
    g_speed_pid.ActualValue = 0.0;      /* �趨Ŀ��Desired Value */
    g_speed_pid.Ui = 0.0;
    g_speed_pid.Up = 0.0;
    g_speed_pid.Ud = 0.0;
    g_speed_pid.Error = 0.0;            /* Error[1] */
    g_speed_pid.LastError = 0.0;        /* Error[-1] */
    g_speed_pid.PrevError = 0.0;        /* Error[-2] */
    g_speed_pid.Proportion = S_KP;      /* �������� Proportional Const */
    g_speed_pid.Integral = S_KI;        /* ���ֳ��� Integral Const */
    g_speed_pid.Derivative = S_KD;      /* ΢�ֳ��� Derivative Const */ 
    g_speed_pid.IngMax = 9000;
    g_speed_pid.IngMin = -9000;
    g_speed_pid.OutMax = 9000;          /* ������� */
    g_speed_pid.OutMin = -9000;
}


/**
 * @brief       λ��ʽPID�㷨
 * @param       *PID��PID�ṹ��������Ӧ��Ŀ��ֵ
 * @param       Feedback_value �� ʵ��ֵ
 * @retval      Ŀ�������
 */
int32_t increment_pid_ctrl(PID_TypeDef *PID,float Feedback_value)
{
    PID->Error = (float)(PID->SetPoint - Feedback_value);   /* Ŀ��ֵ��ʵ��ֵ��ƫ��ֵ */
    PID->Up = PID->Proportion * PID->Error;
    PID->Ui += (PID->Error * PID->Integral);
    LIMIT_OUT(PID->Ui,PID->IngMax,PID->IngMin);             /* �������� */
    PID->Ud = PID->Derivative * (PID->Error - PID->LastError);
    PID->ActualValue = PID->Up + PID->Ui + PID->Ud;
    LIMIT_OUT(PID->ActualValue,PID->OutMax,PID->OutMin);    /* ������� */
    PID->LastError = PID->Error;                            /* �洢�ϴ����Ա��´μ���ʹ�� */
    return ((int32_t)(PID->ActualValue));                   /* ����ʵ�ʿ�����ֵ */
}

///**
// * @brief       �ջ�PID�����㷨���
// * @note        ͨ���� INCR_LOCT_SELECT ѡ��ʹ��λ��ʽ�㷨/����ʽ�㷨
// * @param       *PID��PID�ṹ��������Ӧ��Ŀ��ֵ
// * @param       Feedback_value �� ʵ��ֵ
// * @retval      Ŀ�������
// */
//int32_t increment_pid_ctrl(PID_TypeDef *PID,float Feedback_value)
//{
//    PID->Error = (float)(PID->SetPoint - Feedback_value);                   /* �ٶȵ�λƫ��*/
//#if  INCR_LOCT_SELECT
//    PID->ActualValue += (PID->Proportion * (PID->Error - PID->LastError))   /* E[k]��*/
//                        + (PID->Integral * PID->Error)                      /* E[k-1]��*/
//                        + (PID->Derivative * (PID->Error - 2 * PID->LastError + PID->PrevError)); /*E[k-2]��*/
//    PID->PrevError = PID->LastError;                                        /* �洢�������´μ���*/
//    PID->LastError = PID->Error;
//#else
//    PID->SumError += PID->Error;
//    PID->ActualValue = (PID->Proportion * PID->Error)                       /* E[k]��*/
//                       + (PID->Integral * PID->SumError)                    /* E[k-1]��*/
//                       + (PID->Derivative * (PID->Error - PID->LastError)); /* E[k-2]��*/
//    PID->LastError = PID->Error;
//#endif
//    return ((int32_t)(PID->ActualValue));                                   /* ����ʵ�ʿ�����ֵ*/
//}
