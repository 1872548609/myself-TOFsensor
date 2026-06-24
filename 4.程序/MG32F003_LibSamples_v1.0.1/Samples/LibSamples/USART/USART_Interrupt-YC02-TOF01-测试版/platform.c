/**
 * @file platform.c
 * @brief YC02 TOF01 产品版的板级初始化、按键、灯和 NPN 输出驱动。
 */
#define _PLATFORM_C_

#include "platform.h"

/* VM4304 的 PWD 运行电平，现场若实测相反，仅修改此宏。 */
#define TOF_PWD_RUN_LEVEL             Bit_SET

/* 已按验证测试版保留的管脚定义。 */
#define PLATFORM_LED_GREEN_PIN        GPIO_Pin_10
#define PLATFORM_LED_RED_PIN          GPIO_Pin_9
#define PLATFORM_TOF_PWD_PIN          GPIO_Pin_15
#define PLATFORM_OUTPUT_PIN           GPIO_Pin_11
#define PLATFORM_KEY1_PIN             GPIO_Pin_7
#define PLATFORM_KEY2_PIN             GPIO_Pin_8

volatile uint32_t PLATFORM_DelayTick = 0U;
volatile uint32_t PLATFORM_SystemTickMs = 0U;

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

    /* SysTick 比 USART1/USART2 优先级低，避免破坏 460800bps 收帧。 */
    NVIC_SetPriority(SysTick_IRQn, 0x03U);
}

void PLATFORM_DelayMS(uint32_t Millisecond)
{
    PLATFORM_DelayTick = Millisecond;

    while (PLATFORM_DelayTick != 0U)
    {
    }
}

uint32_t PLATFORM_GetTickMs(void)
{
    return PLATFORM_SystemTickMs;
}

static void PLATFORM_InitBoardGpio(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_GPIOA, ENABLE);

    /*
     * PA9  红灯，PA10 绿灯，PA11 NPN 输出，PA15 TOF PWD。
     * 均为推挽输出。PA11 默认低，保证 Q1 上电截止。
     */
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin = PLATFORM_LED_RED_PIN |
                               PLATFORM_LED_GREEN_PIN |
                               PLATFORM_OUTPUT_PIN |
                               PLATFORM_TOF_PWD_PIN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_WriteBit(GPIOA, PLATFORM_LED_GREEN_PIN, Bit_RESET);
    GPIO_WriteBit(GPIOA, PLATFORM_LED_RED_PIN, Bit_RESET);
    GPIO_WriteBit(GPIOA, PLATFORM_OUTPUT_PIN, Bit_RESET);
    GPIO_WriteBit(GPIOA, PLATFORM_TOF_PWD_PIN, TOF_PWD_RUN_LEVEL);

    /* PA7 / PA8：硬件 10K 上拉，按下接地。 */
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin = PLATFORM_KEY1_PIN | PLATFORM_KEY2_PIN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void PLATFORM_LED_Enable(LEDn_TypeDef LEDn, FunctionalState State)
{
    switch (LEDn)
    {
        case LED1:
            GPIO_WriteBit(GPIOA,
                          PLATFORM_LED_GREEN_PIN,
                          (State == ENABLE) ? Bit_SET : Bit_RESET);
            break;

        case LED2:
            GPIO_WriteBit(GPIOA,
                          PLATFORM_LED_RED_PIN,
                          (State == ENABLE) ? Bit_SET : Bit_RESET);
            break;

        default:
            break;
    }
}

void PLATFORM_LED_Toggle(LEDn_TypeDef LEDn)
{
    switch (LEDn)
    {
        case LED1:
            GPIO_WriteBit(GPIOA,
                          PLATFORM_LED_GREEN_PIN,
                          (GPIO_ReadOutputDataBit(GPIOA,
                                                   PLATFORM_LED_GREEN_PIN) != 0U) ? Bit_RESET : Bit_SET);
            break;

        case LED2:
            GPIO_WriteBit(GPIOA,
                          PLATFORM_LED_RED_PIN,
                          (GPIO_ReadOutputDataBit(GPIOA,
                                                   PLATFORM_LED_RED_PIN) != 0U) ? Bit_RESET : Bit_SET);
            break;

        default:
            break;
    }
}

uint8_t PLATFORM_KeyIsPressed(PLATFORM_Key_t key)
{
    uint16_t pin;

    pin = (key == PLATFORM_KEY1) ? PLATFORM_KEY1_PIN : PLATFORM_KEY2_PIN;

    return (GPIO_ReadInputDataBit(GPIOA, pin) == Bit_RESET) ? 1U : 0U;
}

void PLATFORM_OutputTransistor(FunctionalState State)
{
    GPIO_WriteBit(GPIOA,
                  PLATFORM_OUTPUT_PIN,
                  (State == ENABLE) ? Bit_SET : Bit_RESET);
}

void PLATFORM_Init(void)
{
    PLATFORM_InitDelay();
    PLATFORM_InitBoardGpio();
}
