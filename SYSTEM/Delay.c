#include "Delay.h"

static uint8_t fac_us = 0;  

/**
    微秒级精准延时
*/
void Delay_us(uint32_t nus)
{		
    uint32_t temp;             
    
    if (fac_us == 0) 
    {
        // 配置 SysTick 时钟源为 168MHz / 8 = 21MHz
        SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
        fac_us = 168 / 8; // 算出 1us 需要 21 个脉冲周期
    }

    SysTick->LOAD = nus * fac_us;               // 自动重装载值加载
    SysTick->VAL = 0x00;                        // 清空计数器
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk ;  // 开始倒计时
    
    do
    {
        temp = SysTick->CTRL;
    }while((temp & 0x01) && !(temp & (1 << 16))); // 等待计数器减到0
    
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;  // 关闭SysTick定时器
    SysTick->VAL = 0X00;                        // 清空计数器
}

/**
    毫秒级精准延时
  */
void Delay_ms(uint32_t nms)
{	 	  	  
    while (nms > 500)
    {
        Delay_us(500 * 1000); 
        nms -= 500;
    }
    Delay_us(nms * 1000);     
}

/**
    秒级延时
  */
void Delay_s(uint32_t ns)
{
    while (ns--)
    {
        Delay_ms(1000);       
    }
}
