#include "PWM.h"
#include <rtthread.h>
void PWMOUT1_Init()
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOA, ENABLE);  

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1; 
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Period = 5999; 
	TIM_TimeBaseStructure.TIM_Prescaler =35; 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); 
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
	TIM_OCInitStructure.TIM_Pulse=1000;
	
	TIM_OC1Init(TIM3, &TIM_OCInitStructure); 
	TIM_OC2Init(TIM3, &TIM_OCInitStructure); 
	TIM_OC3Init(TIM3, &TIM_OCInitStructure); 
	TIM_OC4Init(TIM3, &TIM_OCInitStructure); 

	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable); 
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable); 
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable); 
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable); 
 
	TIM_Cmd(TIM3, ENABLE); 
	
 //   TIM_CtrlPWMOutputs(TIM3, ENABLE);

}
void PWMIN1_Init()
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1|RCC_APB2Periph_GPIOA, ENABLE);  

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Period = 0xffff; 
	TIM_TimeBaseStructure.TIM_Prescaler =71; 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); 
	
	TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICFilter=0x0;
	TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;
	
	TIM_ICInitStructure.TIM_Channel=TIM_Channel_1;
	TIM_ICInit(TIM1,&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel=TIM_Channel_2;
	TIM_ICInit(TIM1,&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel=TIM_Channel_3;
	TIM_ICInit(TIM1,&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel=TIM_Channel_4;
	TIM_ICInit(TIM1,&TIM_ICInitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
	NVIC_Init(&NVIC_InitStructure);
 
	TIM_Cmd(TIM1, ENABLE); 
	
	TIM_ITConfig(TIM1, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4, ENABLE); 
}
void PWMIN2_Init()
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Period = 0xffff; 
	TIM_TimeBaseStructure.TIM_Prescaler =71; 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); 
	
	TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICFilter=0x0;
	TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;
	
	TIM_ICInitStructure.TIM_Channel=TIM_Channel_1;
	TIM_ICInit(TIM5,&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel=TIM_Channel_2;
	TIM_ICInit(TIM5,&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel=TIM_Channel_3;
	TIM_ICInit(TIM5,&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel=TIM_Channel_4;
	TIM_ICInit(TIM5,&TIM_ICInitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
	NVIC_Init(&NVIC_InitStructure);
 
	TIM_Cmd(TIM5, ENABLE); 
	
	TIM_ITConfig(TIM5, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4, ENABLE); 
}
u32 PWM1_Time,PWM2_Time,PWM3_Time,PWM4_Time,
	PWM5_Time,PWM6_Time,PWM7_Time,PWM8_Time;
u32 pre[8]={0};
u8 rising[8]={0};
void TIM1_CC_IRQHandler(void)
{
	u32 current[4];
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICStructInit(&TIM_ICInitStructure);
	
	rt_interrupt_enter();
	
	if (TIM_GetITStatus(TIM1, TIM_IT_CC1) == SET) {
		/* Clear TIM1 Capture compare interrupt pending bit */
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);

		if (!rising[0]) {
			TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;

			/* Get the Input Capture value */
			pre[0] = TIM_GetCapture1(TIM1);
			rising[0] = 1;

		} else {
			TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;

			//Get the Input Capture value
			current[0] =  TIM_GetCapture1(TIM1);

			if (current[0] > pre[0])
				PWM1_Time =  current[0] - pre[0];
			else if (current[0] < pre[0])
				PWM1_Time = 0xFFFF - pre[0] + current[0] ;

			rising[0] = 0;
		}
		TIM_ICInit(TIM1, &TIM_ICInitStructure);
	}
	if (TIM_GetITStatus(TIM1, TIM_IT_CC2) == SET) {
		/* Clear TIM1 Capture compare interrupt pending bit */
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC2);

		if (!rising[1]) {
			TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;

			/* Get the Input Capture value */
			pre[1] = TIM_GetCapture2(TIM1);
			rising[1] = 1;

		} else {
			TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;

			//Get the Input Capture value
			current[1] =  TIM_GetCapture2(TIM1);

			if (current[1] > pre[1])
				PWM2_Time =  current[1] - pre[1];
			else if (current[1] < pre[1])
				PWM2_Time = 0xFFFF - pre[1] + current[1] ;

			rising[1] = 0;
		}
		TIM_ICInit(TIM1, &TIM_ICInitStructure);
	}
	if (TIM_GetITStatus(TIM1, TIM_IT_CC3) == SET) {
		/* Clear TIM1 Capture compare interrupt pending bit */
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC3);

		if (!rising[2]) {
			TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;

			/* Get the Input Capture value */
			pre[2] = TIM_GetCapture3(TIM1);
			rising[2] = 1;

		} else {
			TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;

			//Get the Input Capture value
			current[2] =  TIM_GetCapture3(TIM1);

			if (current[2] > pre[2])
				PWM3_Time =  current[2] - pre[2];
			else if (current[2] < pre[2])
				PWM3_Time = 0xFFFF - pre[2] + current[2] ;

			rising[2] = 0;
		}
		TIM_ICInit(TIM1, &TIM_ICInitStructure);
	}
	if (TIM_GetITStatus(TIM1, TIM_IT_CC4) == SET) {
		/* Clear TIM1 Capture compare interrupt pending bit */
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC4);

		if (!rising[3]) {
			TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;

			/* Get the Input Capture value */
			pre[3] = TIM_GetCapture4(TIM1);
			rising[3] = 1;

		} else {
			TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;

			//Get the Input Capture value
			current[3] =  TIM_GetCapture4(TIM1);

			if (current[3] > pre[3])
				PWM4_Time =  current[3] - pre[3];
			else if (current[3] < pre[3])
				PWM4_Time = 0xFFFF - pre[3] + current[3] ;

			rising[3] = 0;
		}
		TIM_ICInit(TIM1, &TIM_ICInitStructure);
	}

	rt_interrupt_leave();
}
void TIM5_IRQHandler(void)
{
	u32 current[4];
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICStructInit(&TIM_ICInitStructure);
	
	rt_interrupt_enter();
	
	if (TIM_GetITStatus(TIM5, TIM_IT_CC1) == SET) {
		/* Clear TIM5 Capture compare interrupt pending bit */
		TIM_ClearITPendingBit(TIM5, TIM_IT_CC1);

		if (!rising[4]) {
			TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;

			/* Get the Input Capture value */
			pre[4] = TIM_GetCapture1(TIM5);
			rising[4] = 1;

		} else {
			TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;

			//Get the Input Capture value
			current[0] =  TIM_GetCapture1(TIM5);

			if (current[0] > pre[4])
				PWM5_Time =  current[0] - pre[4];
			else if (current[0] < pre[4])
				PWM5_Time = 0xFFFF - pre[4] + current[0] ;

			rising[4] = 0;
		}
		TIM_ICInit(TIM5, &TIM_ICInitStructure);
	}
	if (TIM_GetITStatus(TIM5, TIM_IT_CC2) == SET) {
		/* Clear TIM5 Capture compare interrupt pending bit */
		TIM_ClearITPendingBit(TIM5, TIM_IT_CC2);

		if (!rising[5]) {
			TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;

			/* Get the Input Capture value */
			pre[5] = TIM_GetCapture2(TIM5);
			rising[5] = 1;

		} else {
			TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;

			//Get the Input Capture value
			current[1] =  TIM_GetCapture2(TIM5);

			if (current[1] > pre[5])
				PWM6_Time =  current[1] - pre[5];
			else if (current[1] < pre[5])
				PWM6_Time = 0xFFFF - pre[5] + current[1] ;

			rising[5] = 0;
		}
		TIM_ICInit(TIM5, &TIM_ICInitStructure);
	}
	if (TIM_GetITStatus(TIM5, TIM_IT_CC3) == SET) {
		/* Clear TIM5 Capture compare interrupt pending bit */
		TIM_ClearITPendingBit(TIM5, TIM_IT_CC3);

		if (!rising[6]) {
			TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;

			/* Get the Input Capture value */
			pre[6] = TIM_GetCapture3(TIM5);
			rising[6] = 1;

		} else {
			TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;

			//Get the Input Capture value
			current[2] =  TIM_GetCapture3(TIM5);

			if (current[2] > pre[6])
				PWM7_Time =  current[2] - pre[6];
			else if (current[2] < pre[6])
				PWM7_Time = 0xFFFF - pre[6] + current[2] ;

			rising[6] = 0;
		}
		TIM_ICInit(TIM5, &TIM_ICInitStructure);
	}
	if (TIM_GetITStatus(TIM5, TIM_IT_CC4) == SET) {
		/* Clear TIM5 Capture compare interrupt pending bit */
		TIM_ClearITPendingBit(TIM5, TIM_IT_CC4);

		if (!rising[7]) {
			TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;

			/* Get the Input Capture value */
			pre[7] = TIM_GetCapture4(TIM5);
			rising[7] = 1;

		} else {
			TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;

			//Get the Input Capture value
			current[3] =  TIM_GetCapture4(TIM5);

			if (current[3] > pre[7])
				PWM8_Time =  current[3] - pre[7];
			else if (current[3] < pre[7])
				PWM8_Time = 0xFFFF - pre[7] + current[3] ;

			rising[7] = 0;
		}
		TIM_ICInit(TIM5, &TIM_ICInitStructure);
	}	
	
	rt_interrupt_leave();
}
