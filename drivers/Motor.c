#include "Motor.h"
#define MAXOUT 2000
#define MINOUT 2000
void Motor_Init(void)
{
	PWMOUT1_Init();
	PWMIN1_Init();
	PWMIN2_Init();
}
void Motor_Set(u16 m1,u16 m2,u16 m3,u16 m4)
{
	Motor_Set1(m1);
	Motor_Set2(m2);
	Motor_Set3(m3);
	Motor_Set4(m4);
}
void Motor_Set1(u16 m)
{
	TIM_SetCompare1(TIM3,MINOUT+((u32)RangeValue(m,0,1000))*MAXOUT/1000);
}
void Motor_Set2(u16 m)
{
	TIM_SetCompare2(TIM3,MINOUT+((u32)RangeValue(m,0,1000))*MAXOUT/1000);
}
void Motor_Set3(u16 m)
{
	TIM_SetCompare3(TIM3,MINOUT+((u32)RangeValue(m,0,1000))*MAXOUT/1000);
}
void Motor_Set4(u16 m)
{
	TIM_SetCompare4(TIM3,MINOUT+((u32)RangeValue(m,0,1000))*MAXOUT/1000);
}
