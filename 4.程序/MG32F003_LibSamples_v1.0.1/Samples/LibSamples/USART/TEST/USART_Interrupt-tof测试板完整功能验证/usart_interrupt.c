#define _USART_INTERRUPT_C_

#include "platform.h"
#include "usart_interrupt.h"

volatile USART_RxTx_TypeDef USART1_RxStruct = {0};
volatile USART_RxTx_TypeDef USART2_RxStruct = {0};

static void USART_ClearRxStruct(volatile USART_RxTx_TypeDef *pRx, uint16_t MaxLength)
{
    uint16_t i;

    if (MaxLength > USART_RX_BUFFER_SIZE)
    {
        MaxLength = USART_RX_BUFFER_SIZE;
    }

    for (i = 0; i < MaxLength; i++)
    {
        pRx->Buffer[i] = 0;
    }

    pRx->Length       = MaxLength;
    pRx->CurrentCount = 0;
    pRx->CompleteFlag = 0;
    pRx->OverflowFlag = 0;
}

void USART1_TOF_Init(uint32_t Baudrate)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    NVIC_InitTypeDef  NVIC_InitStruct;

    RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_USART1, ENABLE);

    /* PA12 -> USART1_TX */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_4);
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_12;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PA3 -> USART1_RX */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_4);
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_3;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    USART_StructInit(&USART_InitStruct);
    USART_InitStruct.USART_BaudRate   = Baudrate;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits   = USART_StopBits_1;
    USART_InitStruct.USART_Parity     = USART_Parity_No;
    USART_InitStruct.USART_Mode       = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStruct);

    USART_ITConfig(USART1, USART_IT_PE, ENABLE);
    USART_ITConfig(USART1, USART_IT_ERR, ENABLE);

    NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 0x01;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    USART_Cmd(USART1, ENABLE);
}

void USART2_RS485_Init(uint32_t Baudrate)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    NVIC_InitTypeDef  NVIC_InitStruct;

    RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_USART2, ENABLE);

    /* PB1 -> USART2_TX */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_2);
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* PA1 -> USART2_RX */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_2);
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    USART_StructInit(&USART_InitStruct);
    USART_InitStruct.USART_BaudRate   = Baudrate;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits   = USART_StopBits_1;
    USART_InitStruct.USART_Parity     = USART_Parity_No;
    USART_InitStruct.USART_Mode       = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStruct);

    USART_ITConfig(USART2, USART_IT_PE, ENABLE);
    USART_ITConfig(USART2, USART_IT_ERR, ENABLE);

    NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 0x01;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    USART_Cmd(USART2, ENABLE);
}

void USART1_RxIdle_Start(uint16_t MaxLength)
{
    USART_ClearRxStruct(&USART1_RxStruct, MaxLength);

    (void)USART_ReceiveData(USART1);
    USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    USART_ClearFlag(USART1, USART_FLAG_RXNE);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
}

void USART2_RxIdle_Start(uint16_t MaxLength)
{
    USART_ClearRxStruct(&USART2_RxStruct, MaxLength);

    (void)USART_ReceiveData(USART2);
    USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    USART_ClearFlag(USART2, USART_FLAG_RXNE);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
}

void USART1_SendByte(uint8_t Data)
{
    USART_SendData(USART1, Data);
    while (RESET == USART_GetFlagStatus(USART1, USART_FLAG_TC))
    {
    }
}

void USART2_SendByte(uint8_t Data)
{
    USART_SendData(USART2, Data);
    while (RESET == USART_GetFlagStatus(USART2, USART_FLAG_TC))
    {
    }
}

void USART1_SendBuffer(const uint8_t *buf, uint16_t len)
{
    uint16_t i;
    for (i = 0; i < len; i++)
    {
        USART1_SendByte(buf[i]);
    }
}

void USART2_SendBuffer(const uint8_t *buf, uint16_t len)
{
    uint16_t i;
    for (i = 0; i < len; i++)
    {
        USART2_SendByte(buf[i]);
    }
}


void USART_Module_Test(void)
{
    static uint8_t key_blue_last = 0;
    static uint8_t key_red_last  = 0;
    uint8_t key_blue_now;
    uint8_t key_red_now;
    uint8_t msg1[] = "TOF USART1 key blue press\r\n";
    uint8_t msg2[] = "RS485 USART2 key red press\r\n";

    USART1_TOF_Init(460800);
    USART2_RS485_Init(115200);

    USART1_RxIdle_Start(64);
    USART2_RxIdle_Start(64);

    while (1)
    {
        key_blue_now = BOARD_KEY_Read(BOARD_KEY_BLUE);
        key_red_now  = BOARD_KEY_Read(BOARD_KEY_RED);

        if ((key_blue_now == 1) && (key_blue_last == 0))
        {
            BOARD_LED_Toggle(BOARD_LED_GREEN);
            BOARD_OUTPUT_Toggle();
            USART1_SendBuffer(msg1, sizeof(msg1) - 1);
            PLATFORM_DelayMS(20);
        }

        if ((key_red_now == 1) && (key_red_last == 0))
        {
            BOARD_LED_Toggle(BOARD_LED_RED);
            USART2_SendBuffer(msg2, sizeof(msg2) - 1);
            PLATFORM_DelayMS(20);
        }

        key_blue_last = key_blue_now;
        key_red_last  = key_red_now;

        /* TOF串口收到数据，绿灯翻转 */
        if (USART1_RxStruct.CompleteFlag)
        {
            BOARD_LED_Toggle(BOARD_LED_GREEN);
            USART1_RxIdle_Start(64);
        }

        /* RS485串口收到数据，红灯翻转 */
        if (USART2_RxStruct.CompleteFlag)
        {
            BOARD_LED_Toggle(BOARD_LED_RED);
            USART2_RxIdle_Start(64);
        }
    }
}
