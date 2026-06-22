#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "stm32f4xx.h"

/* 电机控制模式。 */
typedef enum
{
    MOTOR_MODE_STOP = 0,
    MOTOR_MODE_RUN,
    MOTOR_MODE_SELF_CHECK,
    MOTOR_MODE_OFFSET_SCAN
} MotorMode_t;

/* 第一类：电机正式运行与控制接口。 */
void MotorControl_Init(void);
void MotorControl_Process(void);
void MotorControl_Start(void);
void MotorControl_Stop(void);
void MotorControl_Increase(void);
void MotorControl_Decrease(void);
void MotorControl_Toggle_Direction(void);
void MotorControl_Set_Target_Rpm(int32_t target_rpm);

/* 第二类：霍尔关系识别与换向表调试接口。 */
void MotorControl_HallPhaseSelfCheck(void);
void MotorControl_AutoOffsetScan(void);

/* 第三类：运行状态与测量结果读取接口。 */
uint8_t MotorControl_Is_Running(void);
uint32_t MotorControl_Get_Time_Us(void);
int32_t MotorControl_Get_Target_Rpm(void);
uint32_t MotorControl_Get_Raw_Rpm(void);
uint32_t MotorControl_Get_Filtered_Rpm(void);
uint16_t MotorControl_Get_Duty_x10(void);

#endif
