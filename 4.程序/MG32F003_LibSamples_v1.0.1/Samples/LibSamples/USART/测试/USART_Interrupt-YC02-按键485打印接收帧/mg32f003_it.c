/**
 * @file mg32f003_it.c
 * @brief 中断服务函数。
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

void SysTick_Handler(void)
{
    if (PLATFORM_DelayTick != 0U)
    {
        PLATFORM_DelayTick--;
    }
}

/**
 * @brief USART1 中断：对应 TOF 模块串口。
 */
void USART1_IRQHandler(void)
{
    uint8_t RxData;

    RxData = 0U;

    /*
     * 错误标志出现时，读 DR 释放接收状态。
     * 当前测试程序仅丢弃该异常字节，下一次 IDLE 后由主循环标记帧失败。
     */
    if ((USART_GetITStatus(USART1, USART_IT_PE) != RESET)
     || (USART_GetITStatus(USART1, USART_IT_ERR) != RESET))
    {
        (void)USART_ReceiveData(USART1);
    }

    /* RXNE：逐字节缓存。 */
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        RxData = USART_ReceiveData(USART1);

        if (USART_RxStruct.CompleteFlag == 0U)
        {
            if (USART_RxStruct.CurrentCount < USART_RxStruct.Length)
            {
                USART_RxStruct.Buffer[USART_RxStruct.CurrentCount++] = RxData;
            }
            else
            {
                /* 超过允许长度，结束本帧，主循环会按无效帧处理。 */
                USART_RxStruct.OverflowFlag = 1U;
                USART_RxStruct.CompleteFlag = 1U;
                USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
                USART_ITConfig(USART1, USART_IT_IDLE, DISABLE);
            }
        }
    }

    /*
     * IDLE：采用“先读状态、再读数据”的清除方式。
     * USART_GetITStatus() 已读取状态寄存器，此处再读一次 DR 清除 IDLE。
     */
    if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
    {
        (void)USART_ReceiveData(USART1);

        if ((USART_RxStruct.CompleteFlag == 0U)
         && (USART_RxStruct.CurrentCount > 0U))
        {
            USART_RxStruct.CompleteFlag = 1U;
            USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
            USART_ITConfig(USART1, USART_IT_IDLE, DISABLE);
        }
    }
}
