#ifndef __MOTOR_H
#define __MOTOR_H
#include "stdio.h"	
#include "sys.h" 

void Motor_PWM_Init(u16 arr,u16 psc);
void Car_Run(int Left_Speed, int Right_Speed);
void Car_Turn(void);
void Left_Motor(int PWM);
void Right_Motor(int PWM);

//���ӱպ�
void Clamp_Close(void);
//���Ӵ�
void Clamp_Open(void);
//�����������
void Elevator_Up(void);
//�����������
void Elevator_Down(void);

#endif
