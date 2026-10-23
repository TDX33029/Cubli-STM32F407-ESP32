
#ifndef STM32_DELAY_H
#define STM32_DELAY_H
/******************************************************************************/

#include "main.h"

void DWT_Init(void);
void delay_us(unsigned long nus);
void delay_ms(unsigned short nms);
void systick_CountMode(void);
/******************************************************************************/

#endif

