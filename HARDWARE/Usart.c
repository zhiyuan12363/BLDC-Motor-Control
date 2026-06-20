#include "Usart.h"
#include <stdarg.h>

#define USART1_TX_BUF_LEN 128 // USART1异步发送缓冲区长度

static char usart1_tx_buf[USART1_TX_BUF_LEN]; // 当前待发送数据帧
static volatile uint16_t usart1_tx_len = 0; // 当前数据帧总长度
static volatile uint16_t usart1_tx_index = 0; // 已发送字节位置
static volatile uint8_t usart1_tx_busy = 0; // 发送忙标志

/*
    初始化USART1，发送使用TXE中断，避免阻塞主循环换向。
*/
void Usart1_Init(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    /* PB6、PB7复用为USART1发送和接收引脚。 */
    GPIO_InitTypeDef GPIO_InitTypeStructure; // 串口引脚初始化结构体
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

    GPIO_InitTypeStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitTypeStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitTypeStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitTypeStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitTypeStructure.GPIO_Speed = GPIO_Fast_Speed;
    GPIO_Init(GPIOB, &GPIO_InitTypeStructure);

    USART_InitTypeDef USART_InitStructure; // USART1参数初始化结构体
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART1, &USART_InitStructure);

    NVIC_InitTypeDef NVIC_InitStructure; // USART1中断优先级结构体
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART1, ENABLE);
}

/*
    格式化一帧数据并启动异步发送，上一帧未完成时直接返回。
*/
uint8_t USART1_Async_Printf(const char *fmt, ...)
{
    va_list args; // 可变参数列表
    int len;      // 格式化后的数据帧长度

    if(usart1_tx_busy != 0)
    {
        return 0;
    }

    va_start(args, fmt);
    len = vsprintf(usart1_tx_buf, fmt, args);
    va_end(args);

    if(len <= 0 || len >= USART1_TX_BUF_LEN)
    {
        return 0;
    }

    usart1_tx_index = 0;
    usart1_tx_len = (uint16_t)len;
    usart1_tx_busy = 1;
    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
    return 1;
}

/*
    USART1发送空中断，每次发送一个字节。
*/
void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
    {
        if(usart1_tx_index < usart1_tx_len)
        {
            USART_SendData(USART1, (uint8_t)usart1_tx_buf[usart1_tx_index]);
            usart1_tx_index++;
        }
        else
        {
            USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
            usart1_tx_busy = 0;
        }
    }
}
