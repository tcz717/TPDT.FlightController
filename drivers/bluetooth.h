#ifndef __USART_H
#define __USART_H
#include <stm32f10x.h>
#include "stdio.h"	
#include "sys.h" 
#include "Queue.h"

#define USART_LEN  			256
	  	
extern u8 RecLineFlag;
extern u8 TCFlag;
extern queue USART_RX_BUF;
void uart_init(u32 bound);
void usart_flush(void);
#endif


