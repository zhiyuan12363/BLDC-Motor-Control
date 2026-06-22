#include "Bldc.h"

#define PWM_PERIOD_COUNTS 8400       // 168MHz下产生20kHz PWM的周期计数
#define DUTY_X10_FULL_SCALE 1000     // 100.0%占空比对应的输入值

static volatile uint16_t pwm_compare[3] = {0, 0, 0}; // 三相PWM比较值
static volatile uint8_t active_high_phase = 0; // 当前输出PWM的上桥臂相号

/*
    六步导通：一相上管PWM，另一相下管GPIO常通，第三相悬空。
*/
static void BLDC_Apply_Commutation(uint8_t high_phase, uint8_t low_phase)
{
    uint32_t enable_mask = 0; // 本次需要使能的TIM1通道

    /* 先关闭所有桥臂，防止换相瞬间上下管重叠导通。 */
    TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE |
                    TIM_CCER_CC2E | TIM_CCER_CC2NE |
                    TIM_CCER_CC3E | TIM_CCER_CC3NE);
    GPIO_ResetBits(GPIOB, GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);

    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;

    /* 选择一个上桥臂输出PWM。 */
    switch(high_phase)
    {
        case 1:
            TIM1->CCR1 = pwm_compare[0];
            enable_mask = TIM_CCER_CC1E;
            break;
        case 2:
            TIM1->CCR2 = pwm_compare[1];
            enable_mask = TIM_CCER_CC2E;
            break;
        case 3:
            TIM1->CCR3 = pwm_compare[2];
            enable_mask = TIM_CCER_CC3E;
            break;
        default:
            active_high_phase = 0;
            return;
    }

    /* 选择另一个相的下桥臂常导通。 */
    switch(low_phase)
    {
        case 1: GPIO_SetBits(GPIOB, GPIO_Pin_13); break;
        case 2: GPIO_SetBits(GPIOB, GPIO_Pin_14); break;
        case 3: GPIO_SetBits(GPIOB, GPIO_Pin_15); break;
        default:
            active_high_phase = 0;
            return;
    }

    active_high_phase = high_phase;
    TIM1->CCER |= enable_mask;
}

/*
    初始化20kHz上桥臂PWM和下桥臂GPIO输出。
*/
void BLDC_PWM_Init(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    /* PA8、PA9、PA10复用为TIM1三相上桥臂PWM输出。 */
    GPIO_InitTypeDef GPIO_InitTypeStructure; // GPIO初始化结构体
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1);

    GPIO_InitTypeStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitTypeStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitTypeStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitTypeStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitTypeStructure.GPIO_Speed = GPIO_High_Speed;
    GPIO_Init(GPIOA, &GPIO_InitTypeStructure);

    /* PB13、PB14、PB15作为三相下桥臂常导通控制端。 */
    GPIO_InitTypeStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitTypeStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOB, &GPIO_InitTypeStructure);
    GPIO_ResetBits(GPIOB, GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);

    /* TIM1以168MHz计数，产生20kHz PWM。 */
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure; // TIM1时基结构体
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = PWM_PERIOD_COUNTS - 1;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 0;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

    TIM_OCInitTypeDef TIM_OCInitStructure; // PWM输出比较结构体
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);

    /* 通道4仅用于PWM周期中点触发霍尔采样中断。 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse = PWM_PERIOD_COUNTS / 2;
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);

    /* 配置高级定时器输出和死区。 */
    TIM_BDTRInitTypeDef TIM_BDTRInitStructure; // 死区和主输出结构体
    TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
    TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
    TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
    TIM_BDTRInitStructure.TIM_DeadTime = 42;
    TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
    TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
    TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
    TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);

    TIM_ClearITPendingBit(TIM1, TIM_IT_CC4);
    TIM_ITConfig(TIM1, TIM_IT_CC4, ENABLE);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitTypeDef NVIC_InitStructure; // TIM1中断优先级结构体
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

/*
    设置三个上桥臂的PWM占空比，单位为0.1%。
*/
void BLDC_Set_PWM_Duty_x10(uint16_t duty1_x10,
                           uint16_t duty2_x10,
                           uint16_t duty3_x10)
{
    if(duty1_x10 > DUTY_X10_FULL_SCALE) duty1_x10 = DUTY_X10_FULL_SCALE;
    if(duty2_x10 > DUTY_X10_FULL_SCALE) duty2_x10 = DUTY_X10_FULL_SCALE;
    if(duty3_x10 > DUTY_X10_FULL_SCALE) duty3_x10 = DUTY_X10_FULL_SCALE;

    //把占空比转换成 CCR 值
    pwm_compare[0] = (uint16_t)(((uint32_t)duty1_x10 * PWM_PERIOD_COUNTS + 500) /
                                DUTY_X10_FULL_SCALE);
    pwm_compare[1] = (uint16_t)(((uint32_t)duty2_x10 * PWM_PERIOD_COUNTS + 500) /
                                DUTY_X10_FULL_SCALE);
    pwm_compare[2] = (uint16_t)(((uint32_t)duty3_x10 * PWM_PERIOD_COUNTS + 500) /
                                DUTY_X10_FULL_SCALE);

    switch(active_high_phase)
    {
        case 1: TIM1->CCR1 = pwm_compare[0]; break;
        case 2: TIM1->CCR2 = pwm_compare[1]; break;
        case 3: TIM1->CCR3 = pwm_compare[2]; break;
        default: break;
    }
}

/*
    强制输出一个上桥PWM、另一下桥常通的导通矢量。
*/
void BLDC_Force_Vector(uint8_t high_phase,
                       uint8_t low_phase,
                       uint16_t duty_x10)
{
    if(high_phase < 1 || high_phase > 3 ||
       low_phase < 1 || low_phase > 3 ||
       high_phase == low_phase || duty_x10 == 0)
    {
        BLDC_Stop();
        return;
    }

    BLDC_Set_PWM_Duty_x10(duty_x10, duty_x10, duty_x10);
    BLDC_Apply_Commutation(high_phase, low_phase);
}

/*
    兼容整数百分比接口。
*/
void BLDC_Set_PWM_Duty(uint16_t duty1, uint16_t duty2, uint16_t duty3)
{
    if(duty1 > 100) duty1 = 100;
    if(duty2 > 100) duty2 = 100;
    if(duty3 > 100) duty3 = 100;

    BLDC_Set_PWM_Duty_x10(duty1 * 10, duty2 * 10, duty3 * 10);
}

/*
    切断所有上下桥臂。
*/
void BLDC_Stop(void)
{
    TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE |
                    TIM_CCER_CC2E | TIM_CCER_CC2NE |
                    TIM_CCER_CC3E | TIM_CCER_CC3NE);
    GPIO_ResetBits(GPIOB, GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    active_high_phase = 0;
}

/*
    按同型号驱动板例程的相序选择一个PWM上桥臂和一个常导通下桥臂。
*/
void BLDC_Commutate(uint8_t hall_state, uint8_t dir)
{
    if(dir == 0)
    {
        switch(hall_state)
        {
            case 2: BLDC_Apply_Commutation(1, 2); break;
            case 6: BLDC_Apply_Commutation(1, 3); break;
            case 4: BLDC_Apply_Commutation(2, 3); break;
            case 5: BLDC_Apply_Commutation(2, 1); break;
            case 1: BLDC_Apply_Commutation(3, 1); break;
            case 3: BLDC_Apply_Commutation(3, 2); break;
            default: BLDC_Stop(); break;
        }
    }
    else
    {
        switch(hall_state)
        {
            case 5: BLDC_Apply_Commutation(1, 2); break;
            case 1: BLDC_Apply_Commutation(1, 3); break;
            case 3: BLDC_Apply_Commutation(2, 3); break;
            case 2: BLDC_Apply_Commutation(2, 1); break;
            case 6: BLDC_Apply_Commutation(3, 1); break;
            case 4: BLDC_Apply_Commutation(3, 2); break;
            default: BLDC_Stop(); break;
        }
    }
}
