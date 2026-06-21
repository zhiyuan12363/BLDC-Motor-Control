#include "MotorControl.h"
#include "Hall.h"
#include "Bldc.h"
#include "Timer.h"
#include "Pid.h"

#define SPEED_CLOSED_LOOP_DEFAULT 1     // 0：开环；1：速度闭环

#define MOTOR_POLE_PAIRS 4              // 电机极对数
#define HALL_EDGES_PER_REV (MOTOR_POLE_PAIRS * 6) // 每机械圈的霍尔有效边沿数
#define SPEED_PERIOD_EDGE_N 6            // 每累计6个同向边沿计算一次转速
#define RPM_PERIOD_COEFF_US ((60000000L * SPEED_PERIOD_EDGE_N) / HALL_EDGES_PER_REV) // 周期法转速系数
#define SPEED_MIN_EDGE_US 300            // 霍尔边沿最小有效间隔
#define SPEED_TIMEOUT_US 300000          // 无霍尔边沿时的转速超时
#define FILTER_N 5                       // 滑动平均窗口长度

#define MOTOR_START_DUTY 10              // 启动阶段占空比，单位1%
#define MOTOR_START_EDGE_N 6             // 启动成功所需连续同向边沿数
#define MOTOR_START_TIMEOUT_US 200000    // 启动超时时间
#define MOTOR_DUTY_MIN 5                 // 开环最小占空比
#define MOTOR_DUTY_MAX 12                // 开环最大占空比
#define TARGET_RPM_MIN 700               // 闭环最低目标转速
#define TARGET_RPM_MAX 3500              // 闭环最高目标转速
#define TARGET_RPM_STEP 100              // 按键单次调整的目标转速

#define HALL_EVENT_NONE 0                // 未发生有效霍尔事件
#define HALL_EVENT_FORWARD 1             // 正向相邻扇区跳变
#define HALL_EVENT_REVERSE 2             // 反向相邻扇区跳变
#define HALL_EVENT_JUMP 3                // 非相邻扇区跳变
#define HALL_EVENT_INITIAL 4             // 首次有效霍尔状态

static volatile uint8_t motor_run_flag = 0; // 电机运行标志
static volatile uint8_t motor_dir = 0; // 0：正转；1：反转
static volatile uint8_t speed_closed_loop_enable = SPEED_CLOSED_LOOP_DEFAULT; // 控制模式
static volatile uint16_t motor_duty = MOTOR_DUTY_MIN; // 开环占空比，单位1%

static volatile uint16_t applied_duty_x10 = 0; // 实际输出占空比，单位0.1%
static volatile uint32_t current_speed_rpm = 0; // 当前周期法转速
static volatile uint32_t filtered_speed_rpm = 0; // 滑动平均后的转速

static volatile uint8_t speed_last_hall = 0; // 测速使用的上一次霍尔状态
static uint8_t speed_edge_count = 0; // 当前测速周期累计边沿数
static volatile uint32_t speed_last_edge_us = 0; // 最近有效边沿时刻
static uint32_t speed_period_start_us = 0; // 当前测速周期起始时刻
static uint32_t speed_filter_buf[FILTER_N] = {0}; // 滑动平均环形缓冲区
static uint32_t speed_filter_sum = 0; // 滑动平均样本总和
static uint8_t speed_filter_index = 0; // 滑动平均写入位置
static uint8_t speed_filter_count = 0; // 当前有效滤波样本数
static uint8_t speed_transition_direction = HALL_EVENT_NONE; // 当前测速方向

static volatile uint8_t startup_active = 0; // 启动过程标志
static volatile uint8_t startup_edge_count = 0; // 启动同向边沿计数
static volatile uint8_t startup_transition_direction = HALL_EVENT_NONE; // 启动检测方向
static uint32_t startup_start_us = 0; // 启动开始时刻
static uint8_t commutation_last_hall = 0; // 上一次换向使用的霍尔状态
static uint32_t commutation_last_edge_us = 0; // 上一次换向边沿时刻

/*
    TIM5作为1us自由运行计时器，不开启中断。
*/
static void Speed_Clock_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure; // TIM5时基初始化结构体
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 84 - 1;
    TIM_TimeBaseInitStructure.TIM_Period = 0xFFFFFFFF;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStructure);

    TIM_Cmd(TIM5, ENABLE);
}

/*
    判断霍尔状态是否属于六步换相的有效状态。
*/
static uint8_t Hall_State_Is_Valid(uint8_t hall_state)
{
    return (hall_state >= 1 && hall_state <= 6);
}

/*
    返回正向霍尔序列中的下一个状态。
*/
static uint8_t Hall_Get_Forward_Next(uint8_t hall_state)
{
    switch(hall_state)
    {
        case 6: return 2;
        case 2: return 3;
        case 3: return 1;
        case 1: return 5;
        case 5: return 4;
        case 4: return 6;
        default: return 0;
    }
}

/*
    返回反向霍尔序列中的下一个状态。
*/
static uint8_t Hall_Get_Reverse_Next(uint8_t hall_state)
{
    switch(hall_state)
    {
        case 6: return 4;
        case 4: return 5;
        case 5: return 1;
        case 1: return 3;
        case 3: return 2;
        case 2: return 6;
        default: return 0;
    }
}

/*
    过滤非法或过短边沿，并判断正向、反向和跳步转换。
*/
static uint8_t Hall_Commutation_Update(uint8_t hall_state, uint32_t now_us)
{
    uint32_t edge_us; // 本次与上次换向边沿的间隔
    uint8_t event; // 本次霍尔跳变类型

    /* 非法霍尔状态立即关闭桥臂。 */
    if(!Hall_State_Is_Valid(hall_state))
    {
        BLDC_Stop();
        commutation_last_hall = 0;
        return HALL_EVENT_NONE;
    }

    if(commutation_last_hall == 0)
    {
        BLDC_Commutate(hall_state, motor_dir);
        commutation_last_hall = hall_state;
        commutation_last_edge_us = now_us;
        return HALL_EVENT_INITIAL;
    }

    if(hall_state == commutation_last_hall)
    {
        return HALL_EVENT_NONE;
    }

    /* 拒绝间隔过短的毛刺，避免错误换相。 */
    edge_us = now_us - commutation_last_edge_us;
    if(edge_us < SPEED_MIN_EDGE_US)
    {
        return HALL_EVENT_NONE;
    }

    /* 按已测得的六状态序列判断转向或跳步。 */
    if(hall_state == Hall_Get_Forward_Next(commutation_last_hall))
    {
        event = HALL_EVENT_FORWARD;
    }
    else if(hall_state == Hall_Get_Reverse_Next(commutation_last_hall))
    {
        event = HALL_EVENT_REVERSE;
    }
    else
    {
        event = HALL_EVENT_JUMP;
    }

    /* 检查完成后再按当前霍尔状态执行换相。 */
    BLDC_Commutate(hall_state, motor_dir);
    commutation_last_hall = hall_state;
    commutation_last_edge_us = now_us;
    return event;
}

/*
    清空滑动平均缓存。
*/
static void Speed_Filter_Reset(void)
{
    uint8_t i; // 滤波缓冲区清零索引

    speed_filter_sum = 0;
    speed_filter_index = 0;
    speed_filter_count = 0;

    for(i = 0; i < FILTER_N; i++)
    {
        speed_filter_buf[i] = 0;
    }
}

/*
    清空测速和滑动平均状态。
*/
static void Speed_Reset(void)
{
    current_speed_rpm = 0;
    filtered_speed_rpm = 0;
    speed_last_hall = 0;
    speed_edge_count = 0;
    speed_last_edge_us = 0;
    speed_period_start_us = 0;
    speed_transition_direction = HALL_EVENT_NONE;
    Speed_Filter_Reset();
}

/*
    5点滑动平均，启动阶段按已有样本数量求平均。
*/
static uint32_t Speed_Moving_Average(uint32_t new_speed)
{
    if(speed_filter_count < FILTER_N)
    {
        speed_filter_buf[speed_filter_index] = new_speed;
        speed_filter_sum += new_speed;
        speed_filter_count++;
    }
    else
    {
        speed_filter_sum -= speed_filter_buf[speed_filter_index];
        speed_filter_buf[speed_filter_index] = new_speed;
        speed_filter_sum += new_speed;
    }

    speed_filter_index++;
    if(speed_filter_index >= FILTER_N)
    {
        speed_filter_index = 0;
    }

    return speed_filter_sum / speed_filter_count;
}

/*
    只累计连续同方向的6个霍尔跳变，往返摆动不计为转速。
*/
static void Speed_Update(uint8_t hall_state, uint32_t now_us, uint8_t event)
{
    uint32_t period_us; // 连续6个同向霍尔边沿的总时间

    if(event == HALL_EVENT_INITIAL)
    {
        speed_last_hall = hall_state;
        speed_last_edge_us = now_us;
        speed_period_start_us = now_us;
        return;
    }

    /* 跳步会破坏周期连续性，重新开始测速和滤波。 */
    if(event == HALL_EVENT_JUMP)
    {
        speed_last_hall = hall_state;
        speed_last_edge_us = now_us;
        speed_period_start_us = now_us;
        speed_edge_count = 0;
        speed_transition_direction = HALL_EVENT_NONE;
        current_speed_rpm = 0;
        filtered_speed_rpm = 0;
        Speed_Filter_Reset();
        return;
    }

    if(event != HALL_EVENT_FORWARD && event != HALL_EVENT_REVERSE)
    {
        return;
    }

    speed_last_hall = hall_state;
    speed_last_edge_us = now_us;

    /* 方向改变时重新建立同方向测速周期，往返摆动不计为转速。 */
    if(speed_transition_direction != event)
    {
        speed_transition_direction = event;
        speed_edge_count = 0;
        speed_period_start_us = now_us;
        current_speed_rpm = 0;
        filtered_speed_rpm = 0;
        Speed_Filter_Reset();
        return;
    }

    speed_edge_count++;

    /* 累计6个同向边沿后，用总周期计算机械转速。 */
    if(speed_edge_count >= SPEED_PERIOD_EDGE_N)
    {
        period_us = now_us - speed_period_start_us;
        speed_period_start_us = now_us;
        speed_edge_count = 0;

        if(period_us != 0)
        {
            current_speed_rpm = (RPM_PERIOD_COEFF_US + period_us / 2) / period_us;
            filtered_speed_rpm = Speed_Moving_Average(current_speed_rpm);
        }
    }
}

/*
    超过300ms没有霍尔变化时，将转速清零。
*/
static uint8_t Speed_Check_Timeout(uint32_t now_us)
{
    if((speed_last_hall != 0) &&
       ((int32_t)(now_us - speed_last_edge_us) > (int32_t)SPEED_TIMEOUT_US))
    {
        Speed_Reset();
        return 1;
    }

    return 0;
}

/*
    根据已测得的5%约700RPM、11%约3750RPM给PI提供启动前馈。
*/
static int32_t Speed_Target_Feedforward_x10(int32_t target_rpm)
{
    if(target_rpm <= 700)
    {
        return 50;
    }
    if(target_rpm >= 3750)
    {
        return 110;
    }

    return 50 + (target_rpm - 700) * 60 / (3750 - 700);
}

/*
    初始化测速时钟、PI和电机控制状态。
*/
void MotorControl_Init(void)
{
    Speed_Clock_Init();
    PID_Init();
    MotorControl_Stop();
}

/*
    启动时短暂施加10%占空比，转过6个霍尔扇区后进入设定模式。
*/
void MotorControl_Start(void)
{
    uint8_t hall_state; // 启动前的转子霍尔位置
    uint8_t event; // 首次换相事件

    if(motor_run_flag != 0)
    {
        return;
    }

    hall_state = Read_Hall_State();
    Speed_Reset();
    startup_active = 1;
    startup_edge_count = 0;
    startup_transition_direction = HALL_EVENT_NONE;
    startup_start_us = TIM5->CNT;
    commutation_last_hall = 0;
    commutation_last_edge_us = startup_start_us;

    PID_Reset(Speed_Target_Feedforward_x10(MotorSpeedPID.target_val));

    applied_duty_x10 = MOTOR_START_DUTY * 10;
    BLDC_Set_PWM_Duty_x10(applied_duty_x10,
                         applied_duty_x10,
                         applied_duty_x10);

    event = Hall_Commutation_Update(hall_state, startup_start_us);
    if(event != HALL_EVENT_NONE)
    {
        Speed_Update(hall_state, startup_start_us, event);
    }

    motor_run_flag = 1;
}

/*
    切断桥臂并清空启动、测速和PI状态。
*/
void MotorControl_Stop(void)
{
    motor_run_flag = 0;
    startup_active = 0;
    startup_transition_direction = HALL_EVENT_NONE;
    commutation_last_hall = 0;
    applied_duty_x10 = 0;
    BLDC_Stop();
    Speed_Reset();
    PID_Reset(Speed_Target_Feedforward_x10(MotorSpeedPID.target_val));
}

/*
    闭环模式增加100RPM，开环模式增加1%占空比。
*/
void MotorControl_Increase(void)
{
    if(motor_run_flag == 0)
    {
        return;
    }

    if(speed_closed_loop_enable != 0)
    {
        if(MotorSpeedPID.target_val < TARGET_RPM_MAX)
        {
            MotorSpeedPID.target_val += TARGET_RPM_STEP;
            if(MotorSpeedPID.target_val > TARGET_RPM_MAX)
            {
                MotorSpeedPID.target_val = TARGET_RPM_MAX;
            }
        }
    }
    else
    {
        if(motor_duty < MOTOR_DUTY_MAX)
        {
            motor_duty++;
        }

        if(startup_active == 0)
        {
            applied_duty_x10 = motor_duty * 10;
            BLDC_Set_PWM_Duty_x10(applied_duty_x10,
                                 applied_duty_x10,
                                 applied_duty_x10);
        }
    }
}

/*
    闭环模式降低100RPM，开环模式降低1%占空比。
*/
void MotorControl_Decrease(void)
{
    if(motor_run_flag == 0)
    {
        return;
    }

    if(speed_closed_loop_enable != 0)
    {
        if(MotorSpeedPID.target_val > TARGET_RPM_MIN)
        {
            MotorSpeedPID.target_val -= TARGET_RPM_STEP;
            if(MotorSpeedPID.target_val < TARGET_RPM_MIN)
            {
                MotorSpeedPID.target_val = TARGET_RPM_MIN;
            }
        }
    }
    else
    {
        if(motor_duty > MOTOR_DUTY_MIN)
        {
            motor_duty--;
        }

        if(startup_active == 0)
        {
            applied_duty_x10 = motor_duty * 10;
            BLDC_Set_PWM_Duty_x10(applied_duty_x10,
                                 applied_duty_x10,
                                 applied_duty_x10);
        }
    }
}

/*
    设置闭环目标转速，并限制在允许范围内。
*/
void MotorControl_Set_Target_Rpm(int32_t target_rpm)
{
    if(target_rpm < TARGET_RPM_MIN)
    {
        target_rpm = TARGET_RPM_MIN;
    }
    else if(target_rpm > TARGET_RPM_MAX)
    {
        target_rpm = TARGET_RPM_MAX;
    }

    MotorSpeedPID.target_val = target_rpm;
}

/*
    仅在停机状态切换电机方向。
*/
void MotorControl_Toggle_Direction(void)
{
    if(motor_run_flag == 0)
    {
        motor_dir = !motor_dir;
    }
}

/*
    处理启动超时、测速超时和10ms速度环计算。
*/
void MotorControl_Process(void)
{
    uint32_t now_us = TIM5->CNT; // 本次控制处理时间戳

    if(motor_run_flag != 0)
    {
        if((Speed_Check_Timeout(now_us) != 0) &&
           (speed_closed_loop_enable != 0))
        {
            MotorControl_Stop();
        }

        if((startup_active != 0) &&
           ((uint32_t)(now_us - startup_start_us) >= MOTOR_START_TIMEOUT_US))
        {
            MotorControl_Stop();
        }
    }

    if(speed_control_tick != 0)
    {
        speed_control_tick = 0;

        /* PI只在电机完成启动且处于闭环模式时执行。 */
        if((motor_run_flag != 0) &&
           (startup_active == 0) &&
           (speed_closed_loop_enable != 0))
        {
            MotorSpeedPID.actual_val = (int32_t)filtered_speed_rpm;
            applied_duty_x10 = (uint16_t)PID_Compute_Incremental_x10(&MotorSpeedPID);
            BLDC_Set_PWM_Duty_x10(applied_duty_x10,
                                 applied_duty_x10,
                                 applied_duty_x10);
        }
    }
}

/*
    TIM1在每个PWM周期中点采样霍尔，发现有效边沿后立即换向和更新测速。
*/
void TIM1_CC_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM1, TIM_IT_CC4) != RESET)
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC4);

        if(motor_run_flag != 0)
        {
            uint8_t hall_state = Read_Hall_State(); // PWM周期中点的霍尔状态
            uint8_t event; // 霍尔状态变化类型
            uint32_t now_us = TIM5->CNT; // 当前1us时间戳

            event = Hall_Commutation_Update(hall_state, now_us);
            if(event != HALL_EVENT_NONE)
            {
                Speed_Update(hall_state, now_us, event);

                /* 连续6个同方向边沿后退出启动占空比。 */
                if(startup_active != 0)
                {
                    if(event == HALL_EVENT_FORWARD || event == HALL_EVENT_REVERSE)
                    {
                        if(startup_transition_direction != event)
                        {
                            startup_transition_direction = event;
                            startup_edge_count = 1;
                        }
                        else
                        {
                            startup_edge_count++;
                        }

                        if(startup_edge_count >= MOTOR_START_EDGE_N)
                        {
                            startup_active = 0;
                            if(speed_closed_loop_enable != 0)
                            {
                                applied_duty_x10 = (uint16_t)MotorSpeedPID.output_x10;
                            }
                            else
                            {
                                applied_duty_x10 = motor_duty * 10;
                            }
                            BLDC_Set_PWM_Duty_x10(applied_duty_x10,
                                                 applied_duty_x10,
                                                 applied_duty_x10);
                        }
                    }
                    else if(event == HALL_EVENT_JUMP)
                    {
                        startup_transition_direction = HALL_EVENT_NONE;
                        startup_edge_count = 0;
                    }
                }
            }
        }
    }
}

/*
    返回电机运行状态。
*/
uint8_t MotorControl_Is_Running(void)
{
    return motor_run_flag;
}

/*
    返回TIM5的1us时间戳。
*/
uint32_t MotorControl_Get_Time_Us(void)
{
    return TIM5->CNT;
}

/*
    返回闭环目标转速，开环模式返回0。
*/
int32_t MotorControl_Get_Target_Rpm(void)
{
    if(speed_closed_loop_enable != 0)
    {
        return MotorSpeedPID.target_val;
    }

    return 0;
}

/*
    返回当前未滤波转速。
*/
uint32_t MotorControl_Get_Raw_Rpm(void)
{
    return current_speed_rpm;
}

/*
    返回滑动平均后的反馈转速。
*/
uint32_t MotorControl_Get_Filtered_Rpm(void)
{
    return filtered_speed_rpm;
}

/*
    返回实际输出占空比，单位0.1%。
*/
uint16_t MotorControl_Get_Duty_x10(void)
{
    return applied_duty_x10;
}
