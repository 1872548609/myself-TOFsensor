/**
 * @file mg32f003_it.c
 * @brief 产品版中断服务函数。
 */
#define _MG32F003_IT_C_

#include "platform.h"
#include "usart_interrupt.h"
#include "mg32f003_it.h"

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
    while (1)
    {
    }
}

void SVC_Handler(void)
{
}

void PendSV_Handler(void)
{
}

/* 1ms 统一时间基准：按键消抖、帧超时、确认时间测试均使用它。 */
void SysTick_Handler(void)
{
    PLATFORM_SystemTickMs++;

    if (PLATFORM_DelayTick != 0U)
    {
        PLATFORM_DelayTick--;
    }
}

/* USART1：TOF 模块，RXNE 收字节，IDLE 封帧。 */
void USART1_IRQHandler(void)
{
    uint8_t rx_data;

    rx_data = 0U;

    if ((USART_GetITStatus(USART1, USART_IT_PE) != RESET) ||
        (USART_GetITStatus(USART1, USART_IT_ERR) != RESET))
    {
        (void)USART_ReceiveData(USART1);
    }

    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        rx_data = USART_ReceiveData(USART1);

        if (USART_RxStruct.CompleteFlag == 0U)
        {
            if (USART_RxStruct.CurrentCount < USART_RxStruct.Length)
            {
                USART_RxStruct.Buffer[USART_RxStruct.CurrentCount++] = rx_data;
            }
            else
            {
                USART_RxStruct.OverflowFlag = 1U;
                USART_RxStruct.CompleteFlag = 1U;
                USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
                USART_ITConfig(USART1, USART_IT_IDLE, DISABLE);
            }
        }
    }

    /* USART_GetITStatus 已访问 SR，随后读 DR 清除 IDLE。 */
    if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
    {
        (void)USART_ReceiveData(USART1);

        if ((USART_RxStruct.CompleteFlag == 0U) &&
            (USART_RxStruct.CurrentCount > 0U))
        {
            USART_RxStruct.CompleteFlag = 1U;
            USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
            USART_ITConfig(USART1, USART_IT_IDLE, DISABLE);
        }
    }
}

/* USART2：RS485 工厂调试发送队列。 */
void USART2_IRQHandler(void)
{
    USART2_DebugTxIRQHandler();
}
