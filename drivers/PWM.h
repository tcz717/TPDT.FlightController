#include <stm32f10x.h>
#ifndef _PWM
#define _PWM
extern u32 PWM1_Time,PWM2_Time,PWM3_Time,PWM4_Time,
			PWM5_Time,PWM6_Time,PWM7_Time,PWM8_Time;
void PWMOUT1_Init(void);
void PWMIN1_Init(void);
void PWMIN2_Init(void);

#endif
