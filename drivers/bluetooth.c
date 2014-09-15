#include "sys.h"
#include "usart.h"	 
#include "misc.h"
  
#pragma import(__use_no_semihosting)                        
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout,__stdin;        
void _sys_exit(int x) 
{ 
	x = x; 
} 
u8 Uart_Send_Buffer[256],send_index=0;
u8 TCFlag=0;
int fputc(int ch, FILE *f)
{      
	Uart_Send_Buffer[send_index++]=ch;
	return ch;
}
u8 RecLineFlag;
queue USART_RX_BUF;   

int fgetc(FILE *f) 
{
	unsigned char ch;
	while(queue_empty(&USART_RX_BUF)){}
	queue_dequeue(&USART_RX_BUF,&ch);
    return ch;
} 
void rt_hw_bluetooth_init(){
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	 
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC| RCC_APB2Periph_AFIO, ENABLE);	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
   
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);  


    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		
	NVIC_Init(&NVIC_InitStructure);	
  

	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	

    USART_Init(UART4, &USART_InitStructure);
    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
    USART_Cmd(UART4, ENABLE);       

	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel4_5_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
	NVIC_Init(&NVIC_InitStructure);

	DMA_DeInit(DMA2_Channel5); 
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&UART4->DR);  
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Uart_Send_Buffer;  
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  
	DMA_InitStructure.DMA_BufferSize = 0xff;  
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; 
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;  
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;  
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  
	DMA_Init(DMA2_Channel5,&DMA_InitStructure);  
	DMA_ITConfig(DMA2_Channel5,DMA_IT_TC,ENABLE);

	queue_init(&USART_RX_BUF,USART_LEN);
	
    config.baud_rate = BAUD_RATE_115200;
}
u8 len;
void UART4_IRQHandler(void)
{
	u8 Res;
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
	{
		Res =USART_ReceiveData(UART4);
		queue_enqueue(&USART_RX_BUF,Res);	
		if(len==0)
		{
			len=Res;
		}
		else
		{		
			if(!RecLineFlag && USART_RX_BUF.count==len+1)
			{
				RecLineFlag=1;
				len=0;
			}
		}
     } 
} 
void usart_flush()
{
	if(send_index>2)
	{
		DMA_Cmd(DMA2_Channel5,DISABLE);  
		DMA_SetCurrDataCounter(DMA2_Channel5,send_index);
		USART_DMACmd(UART4,USART_DMAReq_Tx,ENABLE);  
		DMA_Cmd(DMA2_Channel5,ENABLE); 
		TCFlag=0;
	}
}
void DMA2_Channel4_5_IRQHandler(void)  
{  
    DMA_ClearFlag(DMA2_FLAG_TC5);  
    DMA_Cmd(DMA2_Channel5,DISABLE); 
	send_index=0;
	TCFlag=1;
}  
