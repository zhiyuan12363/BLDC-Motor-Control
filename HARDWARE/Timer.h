#ifndef __TIMER_H
#define __TIMER_H

#include "stm32f4xx.h"

extern volatile uint8_t speed_control_tick;

void Timer3_Init(void);

#endif
