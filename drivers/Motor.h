
#ifndef _MOTOR
#define _MOTOR
#include "PID.h"
#include "PWM.h"
void Motor_Init(void);
void Motor_Set(u16,u16,u16,u16);
void Motor_Set1(u16);
void Motor_Set2(u16);
void Motor_Set3(u16);
void Motor_Set4(u16);
#endif
