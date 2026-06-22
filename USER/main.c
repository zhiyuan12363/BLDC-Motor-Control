#include "Key.h"
#include "Hall.h"
#include "Usart.h"
#include "Bldc.h"
#include "Timer.h"
#include "MotorControl.h"

#define VOFA_PERIOD_US 50000 // VOFA数据发送周期

/*
    根据按键命令调用电机控制接口。
*/
static void Process_Key_Command(uint8_t key_cmd)
{
    switch(key_cmd)
    {
        case KEY_RUN:
            MotorControl_Start();
            break;

        case KEY_STOP:
            MotorControl_Stop();
            break;

        case KEY_UP:
            MotorControl_Increase();
            break;

        case KEY_DOWN:
            MotorControl_Decrease();
            break;

        case KEY_DIR:
            MotorControl_Toggle_Direction();
            break;

        default:
            break;
    }
}

/*
    解析串口命令；V后面的数字在收到换行时设置为目标转速。
*/
static void Process_Serial_Byte(uint8_t data)
{
    static uint32_t target_rpm = 0; // V命令正在接收的目标转速
    static uint8_t target_active = 0; // V命令接收状态
    static uint8_t target_has_digit = 0; // V命令是否已收到数字

    if(target_active != 0)
    {
        if(data >= '0' && data <= '9')
        {
            if(target_rpm < 100000)
            {
                target_rpm = target_rpm * 10 + (uint32_t)(data - '0');
            }
            target_has_digit = 1;
            return;
        }

        if(target_has_digit != 0)
        {
            MotorControl_Set_Target_Rpm((int32_t)target_rpm);
        }

        target_active = 0;
        target_has_digit = 0;
        target_rpm = 0;

        if(data == '\r' || data == '\n')
        {
            return;
        }
    }

    switch(data)
    {
        case 'R':
        case 'r':
            MotorControl_Start();
            break;

        case 'S':
        case 's':
            MotorControl_Stop();
            break;

        case '+':
        case 'U':
        case 'u':
            MotorControl_Increase();
            break;

        case '-':
            MotorControl_Decrease();
            break;

        case 'D':
        case 'd':
            MotorControl_Toggle_Direction();
            break;

        case 'H':
        case 'h':
            if(MotorControl_Is_Running() == 0)
            {
                MotorControl_HallPhaseSelfCheck();
            }
            break;

        case 'O':
        case 'o':
            if(MotorControl_Is_Running() == 0)
            {
                MotorControl_AutoOffsetScan();
            }
            break;

        case 'V':
        case 'v':
            target_active = 1;
            target_has_digit = 0;
            target_rpm = 0;
            break;

        default:
            break;
    }
}

/*
    完成系统初始化，循环处理按键、电机控制和VOFA数据发送。
*/
int main(void)
{
    uint32_t last_vofa_time_us; // 上一次发送VOFA数据的时刻

    Key_Init();
    Hall_Init();
    BLDC_PWM_Init();
    MotorControl_Init();
    Timer3_Init();
    Usart1_Init();

    last_vofa_time_us = MotorControl_Get_Time_Us();

    while(1)
    {
        uint8_t key_cmd = Key_Scan(); // 本次按键命令
        uint8_t serial_data; // 本次串口接收字节
        uint32_t now_us; // 主循环当前时间戳

        Process_Key_Command(key_cmd);
        while(USART1_Read_Byte(&serial_data) != 0)
        {
            Process_Serial_Byte(serial_data);
        }
        MotorControl_Process();
        now_us = MotorControl_Get_Time_Us();

        if((MotorControl_Is_Running() != 0) &&
           ((uint32_t)(now_us - last_vofa_time_us) >= VOFA_PERIOD_US))
        {
            last_vofa_time_us = now_us;
            /* VOFA输出：目标转速、原始转速、滤波转速、占空比（0.1%）。 */
            USART1_Async_Printf("%ld,%lu,%lu,%u\r\n",
                                (long)MotorControl_Get_Target_Rpm(),
                                (unsigned long)MotorControl_Get_Raw_Rpm(),
                                (unsigned long)MotorControl_Get_Filtered_Rpm(),
                                (unsigned int)MotorControl_Get_Duty_x10());
        }
    }
}
