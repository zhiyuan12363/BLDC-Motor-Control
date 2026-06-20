#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f4xx.h"

/* 微秒级延时 */
void Delay_us(uint32_t nus);

/* 毫秒级延时 */
void Delay_ms(uint32_t nms);

/* 秒级延时 */
void Delay_s(uint32_t ns);

#endif
