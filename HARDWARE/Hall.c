#include "Hall.h"

/*
    霍尔传感器GPIO初始化
*/
void Hall_Init(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_InitTypeDef GPIO_InitTypeStructure; // 霍尔输入GPIO初始化结构体
    GPIO_InitTypeStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitTypeStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
    GPIO_InitTypeStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitTypeStructure.GPIO_Speed= GPIO_Fast_Speed;
    GPIO_Init(GPIOC, &GPIO_InitTypeStructure);
}

/*
    读取当前霍尔传感器的组合状态
    返回值为 1~6 的整数，代表电机的 6 个电气位置
*/
uint8_t Read_Hall_State(void)
{
    uint8_t state = 0; // 三路霍尔组合状态

    uint8_t ha = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_6); // 霍尔A电平
    uint8_t hb = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7); // 霍尔B电平
    uint8_t hc = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_8); // 霍尔C电平

    state = (hc << 2)  | (hb << 1) | ha;
    
    return state;
}
