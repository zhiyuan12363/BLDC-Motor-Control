#include "Key.h"

/*
    功能：按键GPIO初始化
*/
void Key_Init(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_InitTypeDef GPIO_InitTypeStructure; // 按键输入GPIO初始化结构体
    GPIO_InitTypeStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitTypeStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_InitTypeStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitTypeStructure.GPIO_Speed = GPIO_Fast_Speed;
    GPIO_Init(GPIOB, &GPIO_InitTypeStructure);

    GPIO_InitTypeStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitTypeStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitTypeStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitTypeStructure.GPIO_Speed = GPIO_Fast_Speed;
    GPIO_Init(GPIOC, &GPIO_InitTypeStructure);
}

/*
    功能：按键扫描
    返回值：按下的键值（0表示无按下）
*/
uint8_t Key_Scan(void)
{
    static uint8_t key_up = 1; // 按键松开后才允许识别下一次按下

    if(key_up && (READ_KEY_RUN == 0 || READ_KEY_STOP == 0 || READ_KEY_UP == 0 || READ_KEY_DOWN == 0 || READ_KEY_DIR == 0))
    {
        /* 换向在主循环执行，按键扫描不能用延时阻塞。 */
        key_up = 0;

        if(READ_KEY_RUN == 0)         return KEY_RUN;
        else if(READ_KEY_STOP == 0)   return KEY_STOP;
        else if(READ_KEY_UP   == 0)   return KEY_UP;
        else if(READ_KEY_DOWN == 0)   return KEY_DOWN;
        else if(READ_KEY_DIR  == 0)   return KEY_DIR;
    }
    else if(READ_KEY_RUN == 1 && READ_KEY_STOP == 1 && READ_KEY_UP == 1 && READ_KEY_DOWN == 1 && READ_KEY_DIR == 1)
    {
        key_up = 1;
    }
    return KEY_NONE;
}
