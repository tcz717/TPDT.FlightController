#include <stm32f10x.h>
#ifndef LED
#define LED
#include "sys.h"
#include <rtthread.h>
void LED_init(void);
void LED_set1(uint16_t state);
void LED_set2(uint16_t state);
void LED_set3(uint16_t state);
void LED_set4(uint16_t state);
#endif
