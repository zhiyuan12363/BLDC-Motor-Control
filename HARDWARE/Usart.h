#ifndef __USART_H
#define __USART_H

#include "stm32f4xx.h"
#include <stdio.h>

void Usart1_Init(void);
uint8_t USART1_Async_Printf(const char *fmt, ...);
uint8_t USART1_Read_Byte(uint8_t *data);

#endif

