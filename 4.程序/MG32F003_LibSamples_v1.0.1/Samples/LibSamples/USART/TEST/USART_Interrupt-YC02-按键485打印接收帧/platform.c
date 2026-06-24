/**
 * @file platform.c
 * @brief YC02 板级基础初始化。
 *
 * 注意：PA10 / PA9 是板载绿灯 / 红灯；PA15 是 TOF PWD。
 * 本测试版本额外将 PA7 / PA8 配置为按键输入。
 */

#define _PLATFORM_C_

#include "platform.h"

/* 若实测 VM4304 的 PWD 为低电平运行，将 Bit_SET 改为 Bit_RESET。 */
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

    /* PA9 红灯、PA10 绿灯、PA15 TOF PWD。 */
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_15;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* 原理图中 LED 经限流电阻接地：高电平点亮。 */
    GPIO_WriteBit(GPIOA, GPIO_Pin_10, Bit_RESET);
    GPIO_WriteBit(GPIOA, GPIO_Pin_9, Bit_RESET);

    /* PA15 为 TOF 模块 PWD。 */
    GPIO_WriteBit(GPIOA, GPIO_Pin_15, TOF_PWD_RUN_LEVEL);

    /*
     * 按键：KEY_BLUE -> PA7，KEY_RED -> PA8。
     * 硬件有 10K 上拉、按下接地，因此按下时读取为低电平。
     * 当前测试仅用 PA7（蓝键）触发 USART2 回传。
     */
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
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
}
