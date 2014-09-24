#include "LED.h"


void LED_init(void)
{
	GPIO_InitTypeDef  initdef;
	initdef.GPIO_Mode=GPIO_Mode_Out_PP;
	initdef.GPIO_Speed=GPIO_Speed_50MHz;
	initdef.GPIO_Pin=GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	GPIO_Init(GPIOC,&initdef);
	
	PCout(12)=1;
	PCout(13)=1;
	PCout(14)=1;
	PCout(15)=1;
}
void LED_set1(uint16_t state)
{
	PCout(12)=state==0;
}
void LED_set2(uint16_t state)
{
	PCout(13)=state==0;
}
void LED_set3(uint16_t state)
{
	PCout(14)=state==0;
}
void LED_set4(uint16_t state)
{
	PCout(15)=state==0;
}
