#ifndef __PID_H
#define __PID_H

#include "stm32f4xx.h"

typedef struct
{
    int32_t target_val;       // 目标转速，单位RPM
    int32_t actual_val;       // 反馈转速，单位RPM
    int32_t err;              // 当前速度误差
    int32_t err_last;         // 上一次速度误差

    float Kp;                 // 比例系数
    float Ki;                 // 积分系数
    float Kd;                 // 微分系数，当前为0

    float output_x10_f;       // 浮点内部输出，单位0.1%
    int32_t output_x10;       // 最终占空比，单位0.1%
    int32_t out_min_x10;      // 最小输出限制
    int32_t out_max_x10;      // 最大输出限制
    int32_t step_up_max_x10;  // 单周期最大输出增加量，单位0.1%
    int32_t step_down_max_x10;// 单周期最大输出减少量，单位0.1%
} PID_TypeDef;

extern PID_TypeDef MotorSpeedPID;

void PID_Init(void);
void PID_Reset(int32_t initial_output_x10);
void PID_Set_Target(int32_t target);
int32_t PID_Compute_Incremental_x10(PID_TypeDef *pid);

#endif
