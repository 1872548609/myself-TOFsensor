/**
 * @file platform.c
 * @brief YC02 板级基础初始化。
 *
 * 注意：原厂 USART 示例把 PA10 当作 USART1 控制台、把 PA15 当作 LED。
 * 本硬件中 PA10 是绿灯、PA15 是 TOF 的 PWD，不能继续使用原示例映射。
 */

#define _PLATFORM_C_

#include "platform.h"

/*
 * 原测试代码因 PLATFORM_InitLED() 将 PA15 当作低有效 LED 使用，
 * 实际上会把 PA15 拉高。本版本显式保留这个上电行为作为 TOF 运行电平。
 * 若实测 VM4304 的 PWD 为低电平运行，只需将 Bit_SET 改为 Bit_RESET。
 */
#define TOF_PWD_RUN_LEVEL    Bit_SET

static void PLATFORM_InitDelay(void)
{
    RCC_ClocksTypeDef RCC_Clocks;

    RCC_GetClocksFreq(&RCC_Clocks);

    if (SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000U))
    {
        while (1)
        {
        }
    }

    NVIC_SetPriority(SysTick_IRQn, 0x00U);
}

void PLATFORM_DelayMS(uint32_t Millisecond)
{
    PLATFORM_DelayTick = Millisecond;

    while (PLATFORM_DelayTick != 0U)
    {
    }
}

static void PLATFORM_InitBoardGpio(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_GPIOA, ENABLE);

    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_15;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* 原理图中 LED 经限流电阻接地：PA10 / PA9 输出高电平时点亮。 */
    GPIO_WriteBit(GPIOA, GPIO_Pin_10, Bit_RESET);
    GPIO_WriteBit(GPIOA, GPIO_Pin_9, Bit_RESET);

    /* PA15 为 TOF 模块 PWD，不再被当作 LED 操作。 */
    GPIO_WriteBit(GPIOA, GPIO_Pin_15, TOF_PWD_RUN_LEVEL);
}

void PLATFORM_LED_Enable(LEDn_TypeDef LEDn, FunctionalState State)
{
    switch (LEDn)
    {
        case LED1:  /* 绿灯：PA10，高有效 */
            GPIO_WriteBit(GPIOA, GPIO_Pin_10,
                          (State == ENABLE) ? Bit_SET : Bit_RESET);
            break;

        case LED2:  /* 红灯：PA9，高有效 */
            GPIO_WriteBit(GPIOA, GPIO_Pin_9,
                          (State == ENABLE) ? Bit_SET : Bit_RESET);
            break;

        case LED3:
        case LED4:
        default:
            /* 本板未使用。 */
            break;
    }
}

void PLATFORM_LED_Toggle(LEDn_TypeDef LEDn)
{
    switch (LEDn)
    {
        case LED1:
            GPIO_WriteBit(GPIOA, GPIO_Pin_10,
                          (GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_10) != 0U)
                              ? Bit_RESET : Bit_SET);
            break;

        case LED2:
            GPIO_WriteBit(GPIOA, GPIO_Pin_9,
                          (GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_9) != 0U)
                              ? Bit_RESET : Bit_SET);
            break;

        case LED3:
        case LED4:
        default:
            break;
    }
}

void PLATFORM_Init(void)
{
    PLATFORM_InitDelay();
    PLATFORM_InitBoardGpio();

    /*
     * 故意不再调用 PLATFORM_InitConsole() / printf：
     * USART1 现在已经连接 TOF，向该口打印调试字符串会直接送到 TOF 模块。
     */
}
