#include "MotorControl.h"
#include "Hall.h"
#include "Bldc.h"
#include "Timer.h"
#include "Pid.h"
#include "Delay.h"
#include "Usart.h"

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

#define SELF_CHECK_DUTY_X10 50            // 自检吸合占空比，单位0.1%
#define SELF_CHECK_HOLD_MS 300            // 每个导通矢量保持时间
#define SELF_CHECK_GAP_MS 500             // 相邻导通矢量之间的断电时间
#define SELF_CHECK_SAMPLE_N 30            // 每个Step的Hall采样次数
#define SELF_CHECK_SAMPLE_MS 2            // 相邻Hall采样间隔
#define SELF_CHECK_MIN_MAJORITY 20        // 接受Hall众数所需最少次数

#define OFFSET_SCAN_DUTY_X10 50           // offset扫描占空比，单位0.1%
#define OFFSET_SCAN_RUN_MS 1200            // 每个offset最长运行时间
#define OFFSET_SCAN_GAP_MS 800             // 相邻offset之间的断电时间
#define OFFSET_SCAN_STALL_MS 300           // 无有效霍尔变化的提前停止时间
#define OFFSET_SCAN_MIN_EDGES 8            // offset通过所需最少正向边沿数
#define OFFSET_SCAN_MAX_JUMPS 3            // 单次测试允许的最大跳步数
#define OFFSET_SCAN_MAX_INVALID 3          // 单次测试允许的最大非法状态数
#define OFFSET_SCAN_FAIL_SCORE (-100000L)  // offset测试失败分数
#define STATIC_STEP_INVALID 0xFF           // 未建立静态映射的Step值

#define HALL_EVENT_NONE 0                // 未发生有效霍尔事件
#define HALL_EVENT_FORWARD 1             // 正向相邻扇区跳变
#define HALL_EVENT_REVERSE 2             // 反向相邻扇区跳变
#define HALL_EVENT_JUMP 3                // 非相邻扇区跳变
#define HALL_EVENT_INITIAL 4             // 首次有效霍尔状态

typedef struct
{
    uint8_t high_phase; // PWM上桥臂相号
    uint8_t low_phase; // 常通下桥臂相号
    const char *name; // 导通矢量名称
} SelfCheckVector_t;

typedef struct
{
    uint8_t offset; // 本次候选Step偏移
    uint16_t forward_count; // 正向相邻霍尔边沿数
    uint16_t reverse_count; // 反向相邻霍尔边沿数
    uint16_t jump_count; // 非相邻霍尔跳变数
    uint16_t invalid_count; // Hall=0或Hall=7出现次数
    uint32_t raw_rpm; // 有效边沿估算转速
    int32_t score; // 综合评分
    uint8_t success; // 本次offset是否通过
} OffsetScanResult_t;

static const SelfCheckVector_t self_vectors[6] =
{
    {1, 2, "U+ V-"},
    {1, 3, "U+ W-"},
    {2, 3, "V+ W-"},
    {2, 1, "V+ U-"},
    {3, 1, "W+ U-"},
    {3, 2, "W+ V-"}
};

static uint8_t hall_to_static_step[8] =
{
    STATIC_STEP_INVALID, STATIC_STEP_INVALID,
    STATIC_STEP_INVALID, STATIC_STEP_INVALID,
    STATIC_STEP_INVALID, STATIC_STEP_INVALID,
    STATIC_STEP_INVALID, STATIC_STEP_INVALID
}; // Hall状态到静态导通Step的映射
static uint8_t step_to_hall[6] = {0, 0, 0, 0, 0, 0}; // 静态Step到Hall状态的映射
static uint8_t hall_seq_forward[6] = {0, 0, 0, 0, 0, 0}; // 自检生成的正向Hall序列

static volatile uint8_t offset_scan_active = 0; // 当前offset测试运行标志
static volatile uint8_t offset_scan_offset = 0; // 当前测试的Step偏移
static volatile uint8_t offset_scan_last_hall = 0; // 最近有效Hall状态
static volatile uint8_t offset_scan_last_sample = 0; // 最近一次原始Hall采样
static volatile uint8_t offset_scan_abort = 0; // 异常提前停止标志
static volatile uint16_t offset_scan_forward_count = 0; // 正向边沿统计
static volatile uint16_t offset_scan_reverse_count = 0; // 反向边沿统计
static volatile uint16_t offset_scan_jump_count = 0; // 跳步统计
static volatile uint16_t offset_scan_invalid_count = 0; // 非法Hall统计
static volatile uint32_t offset_scan_first_edge_us = 0; // 第一个有效边沿时刻
static volatile uint32_t offset_scan_last_edge_us = 0; // 最近有效边沿时刻
static volatile uint32_t offset_scan_last_change_us = 0; // 最近有效Hall变化时刻

static volatile uint8_t motor_run_flag = 0; // 电机运行标志
static volatile MotorMode_t motor_mode = MOTOR_MODE_STOP; // 当前电机控制模式
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

    if(motor_run_flag != 0 || motor_mode != MOTOR_MODE_STOP)
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

    motor_mode = MOTOR_MODE_RUN;
    motor_run_flag = 1;
}

/*
    切断桥臂并清空启动、测速和PI状态。
*/
void MotorControl_Stop(void)
{
    motor_run_flag = 0;
    motor_mode = MOTOR_MODE_STOP;
    offset_scan_active = 0;
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
    清空Hall到静态导通Step的映射。
*/
static void StaticTable_Reset(void)
{
    uint8_t i; // 静态映射清零索引

    for(i = 0; i < 8; i++)
    {
        hall_to_static_step[i] = STATIC_STEP_INVALID;
    }

    for(i = 0; i < 6; i++)
    {
        step_to_hall[i] = 0;
        hall_seq_forward[i] = 0;
    }
}

/*
    连续采样30次Hall并取有效状态众数，少于20票时判定不稳定。
*/
static uint8_t Read_Hall_Stable_For_SelfCheck(void)
{
    uint8_t count[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // Hall 0~7计数
    uint8_t i; // 采样和众数查找索引
    uint8_t hall_state; // 本次Hall采样值
    uint8_t best_state = 0; // 当前众数Hall状态
    uint8_t best_count = 0; // 当前众数出现次数

    for(i = 0; i < SELF_CHECK_SAMPLE_N; i++)
    {
        hall_state = Read_Hall_State();
        if(hall_state <= 7)
        {
            count[hall_state]++;
        }
        Delay_ms(SELF_CHECK_SAMPLE_MS);
    }

    for(i = 0; i < 8; i++)
    {
        if(count[i] > best_count)
        {
            best_count = count[i];
            best_state = i;
        }
    }

    if(best_count < SELF_CHECK_MIN_MAJORITY)
    {
        return 0;
    }

    return best_state;
}

/*
    检查非法、重复和缺失Hall，并生成动态扫描使用的Hall序列。
*/
static uint8_t StaticTable_Validate(void)
{
    uint8_t hall_count[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // 每个Hall出现次数
    uint8_t valid = 1; // 静态映射有效标志
    uint8_t i; // Step和Hall检查索引

    for(i = 0; i < 6; i++)
    {
        if(!Hall_State_Is_Valid(step_to_hall[i]))
        {
            valid = 0;
            while(USART1_Async_Printf("Static Hall table INVALID: Step%u read Hall=%u\r\n",
                                      (unsigned int)i,
                                      (unsigned int)step_to_hall[i]) == 0)
            {
                Delay_ms(1);
            }
        }
        else
        {
            hall_count[step_to_hall[i]]++;
        }
    }

    for(i = 1; i <= 6; i++)
    {
        if(hall_count[i] > 1)
        {
            valid = 0;
            while(USART1_Async_Printf("Static Hall table INVALID: duplicated Hall %u\r\n",
                                      (unsigned int)i) == 0)
            {
                Delay_ms(1);
            }
        }
        else if(hall_count[i] == 0)
        {
            valid = 0;
            while(USART1_Async_Printf("Static Hall table INVALID: missing Hall %u\r\n",
                                      (unsigned int)i) == 0)
            {
                Delay_ms(1);
            }
        }
    }

    if(valid != 0)
    {
        for(i = 0; i < 6; i++)
        {
            hall_seq_forward[i] = step_to_hall[i];
        }
    }

    return valid;
}

/*
    执行六步静态吸合，保存Hall到Step的映射并按需打印每一步。
*/
static uint8_t HallPhase_Capture_Static_Table(uint8_t print_steps)
{
    uint8_t i; // 当前静态导通矢量序号
    uint8_t hall_state; // 当前矢量吸合后的Hall状态

    StaticTable_Reset();

    for(i = 0; i < 6; i++)
    {
        BLDC_Stop();
        Delay_ms(SELF_CHECK_GAP_MS);

        BLDC_Force_Vector(self_vectors[i].high_phase,
                          self_vectors[i].low_phase,
                          SELF_CHECK_DUTY_X10);
        Delay_ms(SELF_CHECK_HOLD_MS);

        hall_state = Read_Hall_Stable_For_SelfCheck();
        BLDC_Stop();

        step_to_hall[i] = hall_state;

        if(Hall_State_Is_Valid(hall_state) &&
           hall_to_static_step[hall_state] == STATIC_STEP_INVALID)
        {
            hall_to_static_step[hall_state] = i;
        }

        if(print_steps != 0)
        {
            while(USART1_Async_Printf("Step%u %s -> Hall=%u\r\n",
                                      (unsigned int)i,
                                      self_vectors[i].name,
                                      (unsigned int)hall_state) == 0)
            {
                Delay_ms(1);
            }
        }
    }

    return StaticTable_Validate();
}

/*
    依次强制输出六个静态导通矢量，并打印转子吸合后的霍尔状态。
*/
void MotorControl_HallPhaseSelfCheck(void)
{
    if(motor_mode != MOTOR_MODE_STOP || motor_run_flag != 0)
    {
        return;
    }

    motor_mode = MOTOR_MODE_SELF_CHECK;
    startup_active = 0;
    applied_duty_x10 = 0;
    speed_control_tick = 0;
    BLDC_Stop();
    Speed_Reset();

    while(USART1_Async_Printf("=== Hall-Phase Self Check ===\r\n") == 0)
    {
        Delay_ms(1);
    }

    HallPhase_Capture_Static_Table(1);

    Delay_ms(20);
    while(USART1_Async_Printf("=== Self Check End ===\r\n") == 0)
    {
        Delay_ms(1);
    }

    BLDC_Stop();
    applied_duty_x10 = 0;
    speed_control_tick = 0;
    Speed_Reset();
    PID_Reset(Speed_Target_Feedforward_x10(MotorSpeedPID.target_val));
    motor_mode = MOTOR_MODE_STOP;
}

/*
    根据Hall静态映射和候选offset输出对应导通矢量。
*/
static uint8_t Apply_Commutation_By_Offset(uint8_t hall_state,
                                            uint8_t offset,
                                            uint16_t duty_x10)
{
    uint8_t static_step; // 当前Hall对应的静态Step
    uint8_t running_step; // 加入offset后的运行Step

    if(!Hall_State_Is_Valid(hall_state) || offset >= 6)
    {
        BLDC_Stop();
        return 0;
    }

    static_step = hall_to_static_step[hall_state];
    if(static_step >= 6)
    {
        BLDC_Stop();
        return 0;
    }

    running_step = (uint8_t)((static_step + offset) % 6);
    BLDC_Force_Vector(self_vectors[running_step].high_phase,
                      self_vectors[running_step].low_phase,
                      duty_x10);
    return 1;
}

/*
    从自检生成的六状态序列中查找下一个Hall状态。
*/
static uint8_t Hall_Get_Next_From_Seq(uint8_t hall_state,
                                      const uint8_t sequence[6])
{
    uint8_t i; // Hall序列查找索引

    for(i = 0; i < 6; i++)
    {
        if(sequence[i] == hall_state)
        {
            return sequence[(i + 1) % 6];
        }
    }

    return 0;
}

/*
    从自检生成的六状态序列中查找上一个Hall状态。
*/
static uint8_t Hall_Get_Prev_From_Seq(uint8_t hall_state,
                                      const uint8_t sequence[6])
{
    uint8_t i; // Hall序列查找索引

    for(i = 0; i < 6; i++)
    {
        if(sequence[i] == hall_state)
        {
            return sequence[(i + 5) % 6];
        }
    }

    return 0;
}

/*
    清空当前offset测试的实时统计量。
*/
static void OffsetScan_Reset_Stats(void)
{
    offset_scan_active = 0;
    offset_scan_abort = 0;
    offset_scan_last_hall = 0;
    offset_scan_last_sample = 0;
    offset_scan_forward_count = 0;
    offset_scan_reverse_count = 0;
    offset_scan_jump_count = 0;
    offset_scan_invalid_count = 0;
    offset_scan_first_edge_us = 0;
    offset_scan_last_edge_us = 0;
    offset_scan_last_change_us = TIM5->CNT;
}

/*
    从当前Hall位置启动一次候选offset测试。
*/
static uint8_t OffsetScan_Begin(uint8_t offset)
{
    uint8_t hall_state; // 测试开始时的Hall状态

    OffsetScan_Reset_Stats();
    offset_scan_offset = offset;
    hall_state = Read_Hall_State();
    offset_scan_last_sample = hall_state;

    if(!Hall_State_Is_Valid(hall_state))
    {
        offset_scan_invalid_count = 1;
        offset_scan_abort = 1;
        BLDC_Stop();
        return 0;
    }

    offset_scan_last_hall = hall_state;
    offset_scan_last_change_us = TIM5->CNT;
    applied_duty_x10 = OFFSET_SCAN_DUTY_X10;

    if(Apply_Commutation_By_Offset(hall_state,
                                   offset_scan_offset,
                                   OFFSET_SCAN_DUTY_X10) == 0)
    {
        offset_scan_abort = 1;
        return 0;
    }

    offset_scan_active = 1;
    return 1;
}

/*
    TIM1中断中的offset扫描换向与Hall顺序统计，不进行串口打印。
*/
static void MotorControl_OffsetScan_Tick(void)
{
    uint8_t hall_state; // 本次PWM中点采样的Hall状态
    uint8_t event; // 本次Hall变化类型
    uint32_t now_us; // 本次Hall采样时间戳

    if(offset_scan_active == 0)
    {
        return;
    }

    hall_state = Read_Hall_State();
    now_us = TIM5->CNT;

    if(!Hall_State_Is_Valid(hall_state))
    {
        if(hall_state != offset_scan_last_sample)
        {
            offset_scan_invalid_count++;
            offset_scan_last_sample = hall_state;
        }

        if(offset_scan_invalid_count > OFFSET_SCAN_MAX_INVALID)
        {
            offset_scan_abort = 1;
            offset_scan_active = 0;
            BLDC_Stop();
        }
        return;
    }

    if(hall_state == offset_scan_last_hall)
    {
        offset_scan_last_sample = hall_state;
        return;
    }

    /* 过短变化按毛刺忽略，保持旧Hall并等待下次采样确认。 */
    if((uint32_t)(now_us - offset_scan_last_change_us) < SPEED_MIN_EDGE_US)
    {
        return;
    }

    offset_scan_last_sample = hall_state;

    if(hall_state == Hall_Get_Next_From_Seq(offset_scan_last_hall,
                                            hall_seq_forward))
    {
        event = HALL_EVENT_FORWARD;
        offset_scan_forward_count++;
    }
    else if(hall_state == Hall_Get_Prev_From_Seq(offset_scan_last_hall,
                                                 hall_seq_forward))
    {
        event = HALL_EVENT_REVERSE;
        offset_scan_reverse_count++;
    }
    else
    {
        event = HALL_EVENT_JUMP;
        offset_scan_jump_count++;
    }

    offset_scan_last_hall = hall_state;
    offset_scan_last_change_us = now_us;

    if(offset_scan_jump_count > OFFSET_SCAN_MAX_JUMPS)
    {
        offset_scan_abort = 1;
        offset_scan_active = 0;
        BLDC_Stop();
        return;
    }

    /* 明显反转时提前停止，避免错误候选表持续驱动。 */
    if(offset_scan_reverse_count >= OFFSET_SCAN_MIN_EDGES &&
       offset_scan_reverse_count > offset_scan_forward_count * 2)
    {
        offset_scan_abort = 1;
        offset_scan_active = 0;
        BLDC_Stop();
        return;
    }

    /* 只有相邻Hall变化才更新候选换向矢量和测速时间。 */
    if(event == HALL_EVENT_FORWARD || event == HALL_EVENT_REVERSE)
    {
        uint16_t valid_edges = offset_scan_forward_count + offset_scan_reverse_count;

        if(valid_edges != 0)
        {
            if(offset_scan_first_edge_us == 0)
            {
                offset_scan_first_edge_us = now_us;
            }
            offset_scan_last_edge_us = now_us;

            if(Apply_Commutation_By_Offset(hall_state,
                                           offset_scan_offset,
                                           OFFSET_SCAN_DUTY_X10) == 0)
            {
                offset_scan_abort = 1;
                offset_scan_active = 0;
            }
        }
    }
}

/*
    根据有效Hall边沿的数量和时间估算机械转速。
*/
static uint32_t OffsetScan_Calculate_Rpm(void)
{
    uint32_t elapsed_us; // 第一个和最后一个有效边沿之间的时间
    uint32_t valid_edges; // 正反向有效边沿总数
    uint64_t numerator; // 64位转速计算分子

    valid_edges = (uint32_t)offset_scan_forward_count +
                  (uint32_t)offset_scan_reverse_count;
    if(valid_edges < 2 || offset_scan_first_edge_us == 0)
    {
        return 0;
    }

    elapsed_us = offset_scan_last_edge_us - offset_scan_first_edge_us;
    if(elapsed_us == 0)
    {
        return 0;
    }

    numerator = (uint64_t)60000000 * (valid_edges - 1);
    return (uint32_t)(numerator /
                      ((uint64_t)HALL_EDGES_PER_REV * elapsed_us));
}

/*
    保存一次offset测试结果并按照Hall连续性和转速评分。
*/
static void OffsetScan_Finish_Result(OffsetScanResult_t *result,
                                     uint8_t offset)
{
    result->offset = offset;
    result->forward_count = offset_scan_forward_count;
    result->reverse_count = offset_scan_reverse_count;
    result->jump_count = offset_scan_jump_count;
    result->invalid_count = offset_scan_invalid_count;
    result->raw_rpm = OffsetScan_Calculate_Rpm();
    result->success = 0;
    result->score = OFFSET_SCAN_FAIL_SCORE;

    if(offset_scan_abort == 0 &&
       result->forward_count >= OFFSET_SCAN_MIN_EDGES &&
       result->jump_count <= OFFSET_SCAN_MAX_JUMPS &&
       result->invalid_count <= OFFSET_SCAN_MAX_INVALID &&
       result->forward_count > result->reverse_count * 2)
    {
        result->success = 1;
        result->score = (int32_t)result->forward_count * 100 +
                        (int32_t)result->raw_rpm -
                        (int32_t)result->reverse_count * 150 -
                        (int32_t)result->jump_count * 300 -
                        (int32_t)result->invalid_count * 500;
    }
}

/*
    自动扫描六种Step偏移，打印评分最高的正转候选换向表。
*/
void MotorControl_AutoOffsetScan(void)
{
    OffsetScanResult_t results[6]; // 六种候选offset测试结果
    int8_t best_offset = -1; // 当前最佳offset，-1表示无通过项
    int32_t best_score = OFFSET_SCAN_FAIL_SCORE; // 当前最佳评分
    uint8_t offset; // 当前测试的offset
    uint8_t hall_state; // 最终表打印使用的Hall状态
    uint32_t test_start_us; // 当前offset测试开始时刻
    uint32_t now_us; // 当前扫描时间戳

    if(motor_mode != MOTOR_MODE_STOP || motor_run_flag != 0)
    {
        return;
    }

    motor_mode = MOTOR_MODE_SELF_CHECK;
    startup_active = 0;
    applied_duty_x10 = 0;
    speed_control_tick = 0;
    BLDC_Stop();
    Speed_Reset();

    while(USART1_Async_Printf("=== Auto Offset Scan ===\r\n") == 0)
    {
        Delay_ms(1);
    }

    if(HallPhase_Capture_Static_Table(1) == 0)
    {
        while(USART1_Async_Printf("Static Hall table incomplete, scan aborted.\r\n") == 0)
        {
            Delay_ms(1);
        }
        while(USART1_Async_Printf("=== Offset Scan End ===\r\n") == 0)
        {
            Delay_ms(1);
        }
        MotorControl_Stop();
        return;
    }

    while(USART1_Async_Printf("Static Hall -> Step table:\r\n") == 0)
    {
        Delay_ms(1);
    }

    for(hall_state = 1; hall_state <= 6; hall_state++)
    {
        uint8_t static_step = hall_to_static_step[hall_state];

        while(USART1_Async_Printf("Hall %u -> Step%u %s\r\n",
                                  (unsigned int)hall_state,
                                  (unsigned int)static_step,
                                  self_vectors[static_step].name) == 0)
        {
            Delay_ms(1);
        }
    }

    while(USART1_Async_Printf("Generated Hall sequence:\r\n") == 0)
    {
        Delay_ms(1);
    }

    while(USART1_Async_Printf("%u -> %u -> %u -> %u -> %u -> %u -> %u\r\n",
                              (unsigned int)hall_seq_forward[0],
                              (unsigned int)hall_seq_forward[1],
                              (unsigned int)hall_seq_forward[2],
                              (unsigned int)hall_seq_forward[3],
                              (unsigned int)hall_seq_forward[4],
                              (unsigned int)hall_seq_forward[5],
                              (unsigned int)hall_seq_forward[0]) == 0)
    {
        Delay_ms(1);
    }

    motor_mode = MOTOR_MODE_OFFSET_SCAN;

    for(offset = 0; offset < 6; offset++)
    {
        while(USART1_Async_Printf("Testing offset %u...\r\n",
                                  (unsigned int)offset) == 0)
        {
            Delay_ms(1);
        }

        BLDC_Stop();
        Delay_ms(OFFSET_SCAN_GAP_MS);
        test_start_us = TIM5->CNT;

        if(OffsetScan_Begin(offset) != 0)
        {
            while(offset_scan_active != 0)
            {
                now_us = TIM5->CNT;

                if(offset_scan_abort != 0)
                {
                    break;
                }

                if((uint32_t)(now_us - test_start_us) >=
                   (uint32_t)OFFSET_SCAN_RUN_MS * 1000)
                {
                    break;
                }

                if((uint32_t)(now_us - offset_scan_last_change_us) >=
                   (uint32_t)OFFSET_SCAN_STALL_MS * 1000)
                {
                    offset_scan_abort = 1;
                    break;
                }

                Delay_ms(1);
            }
        }

        offset_scan_active = 0;
        BLDC_Stop();
        OffsetScan_Finish_Result(&results[offset], offset);

        while(USART1_Async_Printf("Offset %u: F=%u, R=%u, J=%u, INV=%u, RPM=%lu, score=%ld, %s\r\n",
                                  (unsigned int)results[offset].offset,
                                  (unsigned int)results[offset].forward_count,
                                  (unsigned int)results[offset].reverse_count,
                                  (unsigned int)results[offset].jump_count,
                                  (unsigned int)results[offset].invalid_count,
                                  (unsigned long)results[offset].raw_rpm,
                                  (long)results[offset].score,
                                  results[offset].success != 0 ? "PASS" : "FAIL") == 0)
        {
            Delay_ms(1);
        }

        if(results[offset].success != 0 &&
           (best_offset < 0 || results[offset].score > best_score))
        {
            best_offset = (int8_t)offset;
            best_score = results[offset].score;
        }
    }

    motor_mode = MOTOR_MODE_SELF_CHECK;

    if(best_offset >= 0)
    {
        while(USART1_Async_Printf("Best offset = %d\r\n",
                                  (int)best_offset) == 0)
        {
            Delay_ms(1);
        }

        while(USART1_Async_Printf("Final generated-forward commutation table:\r\n") == 0)
        {
            Delay_ms(1);
        }

        for(hall_state = 1; hall_state <= 6; hall_state++)
        {
            uint8_t running_step =
                (uint8_t)((hall_to_static_step[hall_state] + best_offset) % 6);

            while(USART1_Async_Printf("Hall %u -> Step%u %s\r\n",
                                      (unsigned int)hall_state,
                                      (unsigned int)running_step,
                                      self_vectors[running_step].name) == 0)
            {
                Delay_ms(1);
            }
        }

        while(USART1_Async_Printf("Note: generated-forward direction may not match physical clockwise direction. Please verify actual motor direction manually before writing this table into BLDC_Commutate().\r\n") == 0)
        {
            Delay_ms(1);
        }
    }
    else
    {
        while(USART1_Async_Printf("No valid offset found.\r\n") == 0)
        {
            Delay_ms(1);
        }
    }

    while(USART1_Async_Printf("=== Offset Scan End ===\r\n") == 0)
    {
        Delay_ms(1);
    }

    MotorControl_Stop();
}

/*
    处理启动超时、测速超时和10ms速度环计算。
*/
void MotorControl_Process(void)
{
    uint32_t now_us = TIM5->CNT; // 本次控制处理时间戳

    if(motor_mode == MOTOR_MODE_SELF_CHECK ||
       motor_mode == MOTOR_MODE_OFFSET_SCAN)
    {
        speed_control_tick = 0;
        return;
    }

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

        if(motor_mode == MOTOR_MODE_SELF_CHECK)
        {
            return;
        }

        if(motor_mode == MOTOR_MODE_OFFSET_SCAN)
        {
            MotorControl_OffsetScan_Tick();
            return;
        }

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
