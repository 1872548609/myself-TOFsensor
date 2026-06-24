#define _MG32F003_IT_C_

#include "platform.h"
#include "usart_interrupt.h"
#include "mg32f003_it.h"

void NMI_Handler(void) {}
void HardFault_Handler(void) { while (1) {} }
void SVC_Handler(void) {}
void PendSV_Handler(void) {}

void SysTick_Handler(void)
{
    if (0 != PLATFORM_DelayTick)
    {
        PLATFORM_DelayTick--;
    }
}

 static void USART_RxIdle_Handler(USART_TypeDef  *USARTx, volatile USART_RxTx_TypeDef *rx)
{
    uint8_t RxData = 0;

    if ((RESET != USART_GetITStatus(USARTx, USART_IT_PE)) ||
        (RESET != USART_GetITStatus(USARTx, USART_IT_ERR)))
    {
        (void)USART_ReceiveData(USARTx);
    }

    if (RESET != USART_GetITStatus(USARTx, USART_IT_RXNE))
    {
        RxData = USART_ReceiveData(USARTx);

        if (0 == rx->CompleteFlag)
        {
            if (rx->CurrentCount < rx->Length)
            {
                rx->Buffer[rx->CurrentCount++] = RxData;
            }
            else
            {
                rx->OverflowFlag = 1;
                rx->CompleteFlag = 1;

                USART_ITConfig(USARTx, USART_IT_RXNE, DISABLE);
                USART_ITConfig(USARTx, USART_IT_IDLE, DISABLE);
            }
        }
    }

    if (RESET != USART_GetITStatus(USARTx, USART_IT_IDLE))
    {
        (void)USART_ReceiveData(USARTx);

        if ((0 == rx->CompleteFlag) && (rx->CurrentCount > 0))
        {
            rx->CompleteFlag = 1;
            USART_ITConfig(USARTx, USART_IT_RXNE, DISABLE);
            USART_ITConfig(USARTx, USART_IT_IDLE, DISABLE);
        }
    }
}

void USART1_IRQHandler(void)
{
    USART_RxIdle_Handler(USART1, &USART1_RxStruct);
}

void USART2_IRQHandler(void)
{
    USART_RxIdle_Handler(USART2, &USART2_RxStruct);
}
