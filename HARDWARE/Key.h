#ifndef __KEY_H
#define __KEY_H

#include "stm32f4xx.h"

#define KEY_NONE 0 // 无按键命令
#define KEY_RUN  1 // 启动电机
#define KEY_STOP 2 // 停止电机
#define KEY_UP   3 // 增加目标转速或开环占空比
#define KEY_DOWN 4 // 减少目标转速或开环占空比
#define KEY_DIR  5 // 停机状态下切换方向

#define READ_KEY_RUN  GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_5)  // 读取启动键
#define READ_KEY_STOP GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1)  // 读取停止键
#define READ_KEY_UP   GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10) // 读取增加键
#define READ_KEY_DOWN GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) // 读取减少键
#define READ_KEY_DIR  GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12) // 读取方向键

void Key_Init(void);
uint8_t Key_Scan(void);

#endif
