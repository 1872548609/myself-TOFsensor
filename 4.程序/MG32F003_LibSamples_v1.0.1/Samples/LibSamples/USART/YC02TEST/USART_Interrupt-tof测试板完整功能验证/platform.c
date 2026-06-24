/**
 * @file    platform.c
 * @author  MegawinTech Application Team
 * @version V1.0.1
 * @date    17-Nov-2023
 * @brief   This file contains all the system functions
 */

/* Define to prevent recursive inclusion */
#define _PLATFORM_C_

/* Files include */
#include <stdio.h>
#include "platform.h"

/**
  * @addtogroup MG32F003_LibSamples
  * @{
  */

/**
  * @addtogroup MG32
  * @{
  */

/**
  * @addtogroup PLATFORM
  * @{
  */

/* Private typedef ****************************************************************************************************/

/* Private define *****************************************************************************************************/

/* Private macro ******************************************************************************************************/

/* Private variables **************************************************************************************************/

/* Private functions **************************************************************************************************/

/***********************************************************************************************************************
  * @brief  Initialize SysTick for delay function
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void PLATFORM_InitDelay(void)
{
    RCC_ClocksTypeDef RCC_Clocks;

    RCC_GetClocksFreq(&RCC_Clocks);

    if (SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000))
    {
        while (1)
        {
        }
    }

    NVIC_SetPriority(SysTick_IRQn, 0x0);
}

/***********************************************************************************************************************
  * @brief  Millisecond delay
  * @note   none
  * @param  Millisecond: delay time unit
  * @retval none
  *********************************************************************************************************************/
void PLATFORM_DelayMS(uint32_t Millisecond)
{
    PLATFORM_DelayTick = Millisecond;

    while (0 != PLATFORM_DelayTick)
    {
    }
}

/***********************************************************************************************************************
  * @brief  BOARD GPIO FUNCTION
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void BOARD_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_GPIOB, ENABLE);

    /* LED：推挽输出
       从原理图看 LED 另一端接 GND2，默认按高电平点亮处理 */
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin   = LED_GREEN_PIN | LED_RED_PIN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* OUTPUT：推挽输出 */
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin   = OUTPUT_PIN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* KEY：上拉输入
       原理图里按键按下接地，外部10K上拉到VC2，所以按下=0，松开=1 */
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin   = KEY_BLUE_PIN | KEY_RED_PIN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* 默认关闭 */
    BOARD_LED_Off(BOARD_LED_GREEN);
    BOARD_LED_Off(BOARD_LED_RED);
    BOARD_OUTPUT_Off();
}

void BOARD_LED_On(BOARD_LED_TypeDef led)
{
    if (led == BOARD_LED_GREEN)
    {
        GPIO_WriteBit(LED_GREEN_PORT, LED_GREEN_PIN, Bit_SET);
    }
    else
    {
        GPIO_WriteBit(LED_RED_PORT, LED_RED_PIN, Bit_SET);
    }
}

void BOARD_LED_Off(BOARD_LED_TypeDef led)
{
    if (led == BOARD_LED_GREEN)
    {
        GPIO_WriteBit(LED_GREEN_PORT, LED_GREEN_PIN, Bit_RESET);
    }
    else
    {
        GPIO_WriteBit(LED_RED_PORT, LED_RED_PIN, Bit_RESET);
    }
}

void BOARD_LED_Toggle(BOARD_LED_TypeDef led)
{
    if (led == BOARD_LED_GREEN)
    {
        GPIO_WriteBit(LED_GREEN_PORT, LED_GREEN_PIN,
                      GPIO_ReadOutputDataBit(LED_GREEN_PORT, LED_GREEN_PIN) ? Bit_RESET : Bit_SET);
    }
    else
    {
        GPIO_WriteBit(LED_RED_PORT, LED_RED_PIN,
                      GPIO_ReadOutputDataBit(LED_RED_PORT, LED_RED_PIN) ? Bit_RESET : Bit_SET);
    }
}

uint8_t BOARD_KEY_Read(BOARD_KEY_TypeDef key)
{
    if (key == BOARD_KEY_BLUE)
    {
        return (GPIO_ReadInputDataBit(KEY_BLUE_PORT, KEY_BLUE_PIN) == Bit_RESET) ? 1 : 0;
    }
    else
    {
        return (GPIO_ReadInputDataBit(KEY_RED_PORT, KEY_RED_PIN) == Bit_RESET) ? 1 : 0;
    }
}

void BOARD_OUTPUT_On(void)
{
    GPIO_WriteBit(OUTPUT_PORT, OUTPUT_PIN, Bit_SET);
}

void BOARD_OUTPUT_Off(void)
{
    GPIO_WriteBit(OUTPUT_PORT, OUTPUT_PIN, Bit_RESET);
}

void BOARD_OUTPUT_Toggle(void)
{
    GPIO_WriteBit(OUTPUT_PORT, OUTPUT_PIN,
                  GPIO_ReadOutputDataBit(OUTPUT_PORT, OUTPUT_PIN) ? Bit_RESET : Bit_SET);
}

/***********************************************************************************************************************
  * @brief  redefine fputc function
  * @note   for printf
  * @param  ch
  * @param  f
  * @retval ch
  *********************************************************************************************************************/
int fputc(int ch, FILE *f)
{
    USART_SendData(USART1, (uint8_t)ch);

    while (RESET == USART_GetFlagStatus(USART1, USART_FLAG_TC))
    {
    }

    return (ch);
}

/***********************************************************************************************************************
  * @brief  Initialize Platform
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void PLATFORM_Init(void)
{
   PLATFORM_InitDelay();
    BOARD_GPIO_Init();
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

