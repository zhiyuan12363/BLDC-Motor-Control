#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "stm32f4xx.h"

void MotorControl_Init(void);
void MotorControl_Process(void);
void MotorControl_Start(void);
void MotorControl_Stop(void);
void MotorControl_Increase(void);
void MotorControl_Decrease(void);
void MotorControl_Toggle_Direction(void);
void MotorControl_Set_Target_Rpm(int32_t target_rpm);

uint8_t MotorControl_Is_Running(void);
uint32_t MotorControl_Get_Time_Us(void);
int32_t MotorControl_Get_Target_Rpm(void);
uint32_t MotorControl_Get_Raw_Rpm(void);
uint32_t MotorControl_Get_Filtered_Rpm(void);
uint16_t MotorControl_Get_Duty_x10(void);

#endif
