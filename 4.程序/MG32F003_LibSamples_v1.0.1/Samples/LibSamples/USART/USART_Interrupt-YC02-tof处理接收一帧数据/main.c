/**
 * @file main.c
 * @brief YC02 TOF 测试程序入口
 */

#define _MAIN_C_

#include "platform.h"
#include "usart_interrupt.h"
#include "main.h"

int main(void)
{
    PLATFORM_Init();

    /*
     * 本测试函数内部保持循环：
     * - USART1（PA12/PA3）接收 TOF 原始帧；
     * - 通过 PA10 绿灯 / PA9 红灯显示解析状态；
     * - 可在 Keil Watch 中观察 g_tof_xxx 变量。
     */
    USART_Interrupt_Sample();

    while (1)
    {
    }
}
