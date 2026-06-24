/**
 * @file    usart_interrupt.c
 * @author  MegawinTech Application Team
 * @version V1.0.1
 * @date    14-Nov-2023
 * @brief   This file contains all the system functions
 */

/* Define to prevent recursive inclusion */
#define _USART_INTERRUPT_C_

/* Files include */
#include <stdio.h>
#include "platform.h"
#include "usart_interrupt.h"

/**
  * @addtogroup MG32F003_LibSamples
  * @{
  */

/**
  * @addtogroup USART
  * @{
  */

/**
  * @addtogroup USART_Interrupt
  * @{
  */

/* Private typedef ****************************************************************************************************/

/* Private define *****************************************************************************************************/

/* Private macro ******************************************************************************************************/

/* Private variables **************************************************************************************************/
/* 全局变量定义 */
volatile USART_RxTx_TypeDef USART_RxStruct = {0};
volatile USART_RxTx_TypeDef USART_TxStruct = {0};
/* Private functions **************************************************************************************************/

/***********************************************************************************************************************
  * @brief
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void USART_Configure(uint32_t Baudrate)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    NVIC_InitTypeDef  NVIC_InitStruct;
    USART_InitTypeDef USART_InitStruct;

    NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 0x01;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_GPIOB, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_2);

    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    RCC_APB1PeriphClockCmd(RCC_APB1ENR_USART2, ENABLE);

    USART_StructInit(&USART_InitStruct);
    USART_InitStruct.USART_BaudRate   = Baudrate;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits   = USART_StopBits_1;
    USART_InitStruct.USART_Parity     = USART_Parity_No;
    USART_InitStruct.USART_Mode       = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStruct);

    USART_ITConfig(USART2, USART_IT_PE, ENABLE);
    USART_ITConfig(USART2, USART_IT_ERR, ENABLE);

    USART_Cmd(USART2, ENABLE);
}


/* 启动一次 IDLE 帧接收
 * MaxLength = 本帧最多允许接收多少字节，可自定义
 */
void USART_RxIdle_Start(uint16_t MaxLength)
{
    uint16_t i;

    if (MaxLength > USART_RX_BUFFER_SIZE)
    {
        MaxLength = USART_RX_BUFFER_SIZE;
    }

    for (i = 0; i < MaxLength; i++)
    {
        USART_RxStruct.Buffer[i] = 0;
    }

    USART_RxStruct.Length       = MaxLength;
    USART_RxStruct.CurrentCount = 0;
    USART_RxStruct.CompleteFlag = 0;
    USART_RxStruct.OverflowFlag = 0;

    /* 先清一次残留状态 */
    (void)USART_ReceiveData(USART2);
    USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    USART_ClearFlag(USART2, USART_FLAG_RXNE);

    /* 开启接收和空闲中断 */
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
}


/***********************************************************************************************************************
  * @brief
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
//void USART_RxData_Interrupt(uint8_t Length)
//{
//    uint8_t i = 0;

//    for (i = 0; i < Length; i++)
//    {
//        USART_RxStruct.Buffer[i] = 0;
//    }

//    USART_RxStruct.Length = Length;
//    USART_RxStruct.CurrentCount = 0;
//    USART_RxStruct.CompleteFlag = 0;

//    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
//}

/***********************************************************************************************************************
  * @brief
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void USART_TxData_Interrupt(uint8_t *Buffer, uint8_t Length)
{
    uint8_t i = 0;

    for (i = 0; i < Length; i++)
    {
        USART_TxStruct.Buffer[i] = Buffer[i];
    }

    USART_TxStruct.Length = Length;
    USART_TxStruct.CurrentCount = 0;
    USART_TxStruct.CompleteFlag = 0;

    USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
}

/***********************************************************************************************************************
  * @brief
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void USART_Interrupt_Sample(void)
{
    printf("\r\nTest %s", __FUNCTION__);

    USART_RxStruct.CompleteFlag = 0;
    USART_TxStruct.CompleteFlag = 1;

    USART_Configure(115200);

    //USART_RxData_Interrupt(10);
    
    /* 这里自定义一帧最大长度，比如 64 字节 */
    USART_RxIdle_Start(27);

    printf("\r\nSend 10 bytes to USART every time");

     while (1)
    {
        if (USART_RxStruct.CompleteFlag)
        {
            /* 回发本次收到的实际长度 */
            USART_TxData_Interrupt((uint8_t *)USART_RxStruct.Buffer, USART_RxStruct.CurrentCount);

            while (0 == USART_TxStruct.CompleteFlag)
            {
            }

            printf("\r\nFrame done, len = %d", USART_RxStruct.CurrentCount);

            /* 重新开启下一帧接收 */
            USART_RxIdle_Start(27);
        }
    }
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

