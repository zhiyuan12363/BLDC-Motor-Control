#include "Pid.h"

PID_TypeDef MotorSpeedPID; // 电机速度环PI控制器

/*
    初始化10ms周期增量式PI，输出单位为0.1%占空比。
*/
void PID_Init(void)
{
    MotorSpeedPID.target_val = 1000;
    MotorSpeedPID.actual_val = 0;
    MotorSpeedPID.err = 0;
    MotorSpeedPID.err_last = 0;

    MotorSpeedPID.Kp = 0.020f;
    MotorSpeedPID.Ki = 0.0025f;
    MotorSpeedPID.Kd = 0.0f;

    MotorSpeedPID.output_x10_f = 50.0f;
    MotorSpeedPID.output_x10 = 50;
    MotorSpeedPID.out_min_x10 = 50;
    MotorSpeedPID.out_max_x10 = 110;
    MotorSpeedPID.step_up_max_x10 = 2;
    MotorSpeedPID.step_down_max_x10 = 5;
}

/*
    启动或停机后清空历史误差，并指定初始占空比。
*/
void PID_Reset(int32_t initial_output_x10)
{
    if(initial_output_x10 > MotorSpeedPID.out_max_x10)
    {
        initial_output_x10 = MotorSpeedPID.out_max_x10;
    }
    if(initial_output_x10 < MotorSpeedPID.out_min_x10)
    {
        initial_output_x10 = MotorSpeedPID.out_min_x10;
    }

    MotorSpeedPID.actual_val = 0;
    MotorSpeedPID.err = 0;
    MotorSpeedPID.err_last = 0;
    MotorSpeedPID.output_x10_f = (float)initial_output_x10;
    MotorSpeedPID.output_x10 = initial_output_x10;
}

/*
    设置目标转速。
*/
void PID_Set_Target(int32_t target)
{
    MotorSpeedPID.target_val = target;
}

/*
    增量式PI计算，带输出限幅和单周期变化率限制。
*/
int32_t PID_Compute_Incremental_x10(PID_TypeDef *pid)
{
    float delta_x10; // 本控制周期的占空比增量，单位0.1%

    /* 增量式PI：比例项使用误差变化量，积分项使用当前误差。 */
    pid->err = pid->target_val - pid->actual_val;
    delta_x10 = pid->Kp * (float)(pid->err - pid->err_last)
              + pid->Ki * (float)pid->err;

    /* 加载时平缓增加输出，卸载时允许更快降低输出。 */
    if(delta_x10 > (float)pid->step_up_max_x10)
    {
        delta_x10 = (float)pid->step_up_max_x10;
    }
    else if(delta_x10 < -(float)pid->step_down_max_x10)
    {
        delta_x10 = -(float)pid->step_down_max_x10;
    }

    /* 累加增量并限制最终占空比范围。 */
    pid->output_x10_f += delta_x10;
    if(pid->output_x10_f > (float)pid->out_max_x10)
    {
        pid->output_x10_f = (float)pid->out_max_x10;
    }
    else if(pid->output_x10_f < (float)pid->out_min_x10)
    {
        pid->output_x10_f = (float)pid->out_min_x10;
    }

    /* 四舍五入为0.1%分辨率的整数输出，并保存历史误差。 */
    pid->output_x10 = (int32_t)(pid->output_x10_f + 0.5f);
    pid->err_last = pid->err;
    return pid->output_x10;
}
