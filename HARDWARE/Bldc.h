#ifndef __BLDC_H
#define __BLDC_H

#include "stm32f4xx.h"

void BLDC_PWM_Init(void);
void BLDC_Set_PWM_Duty(uint16_t duty1, uint16_t duty2, uint16_t duty3);
void BLDC_Set_PWM_Duty_x10(uint16_t duty1_x10,
                           uint16_t duty2_x10,
                           uint16_t duty3_x10);
void BLDC_Stop(void);
void BLDC_Commutate(uint8_t hall_state, uint8_t dir);

#endif
