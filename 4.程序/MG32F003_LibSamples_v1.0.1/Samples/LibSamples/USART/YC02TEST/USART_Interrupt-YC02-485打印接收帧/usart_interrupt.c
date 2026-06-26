/**
 * @file    usart_interrupt.c
 * @brief   USART1 接收 TOF 原始帧；USART2 经 RS485 输出每一帧十六进制文本
 */

#define _USART_INTERRUPT_C_

#include "platform.h"
#include "usart_interrupt.h"

/* -------------------- 串口参数 -------------------- */
#define TOF_USART_BAUDRATE              460800UL
#define RS485_DEBUG_BAUDRATE            460800UL

/*
 * 输出格式：
 * TOF RX: 5A 8E 17 37 ... 7D\r\n
 *
 * 固定前缀 8 字节，单个原始字节转换为 "XX " 共 3 字节，结尾为 CRLF。
 */
#define RS485_DEBUG_PREFIX_LENGTH       8U
#define RS485_DEBUG_TEXT_MAX_LENGTH     (RS485_DEBUG_PREFIX_LENGTH + \
                                         (TOF_FRAME_MAX_LENGTH * 3U) + 2U)

/* USART1：TOF；USART2：RS485 调试输出。 */
volatile USART_RxTx_TypeDef USART_RxStruct = {0};
volatile USART_RxTx_TypeDef USART_TxStruct = {0};

static const uint8_t s_hex_table[] = "0123456789ABCDEF";

/***********************************************************************************************************************
 * @brief  配置 USART1 为 TOF 通信口
 *
 *         原理图对应：
 *         PA12 / AF1 -> USART1_TX -> TOF_RXD
 *         PA3  / AF1 -> USART1_RX <- TOF_TXD
 ***********************************************************************************************************************/
static void USART1_TOF_Configure(uint32_t Baudrate)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    NVIC_InitTypeDef  NVIC_InitStruct;
    USART_InitTypeDef USART_InitStruct;

    RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1ENR_USART1, ENABLE);

    /* PA12：USART1_TX，AF1。 */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_1);
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_12;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PA3：USART1_RX，AF1。 */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);
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

    NVIC_InitStruct.NVIC_IRQChannel         = USART1_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 0x00;   /* TOF 接收优先级最高 */
    NVIC_InitStruct.NVIC_IRQChannelCmd      = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    USART_Cmd(USART1, ENABLE);
}

/***********************************************************************************************************************
 * @brief  配置 USART2 为 RS485 调试输出口
 *
 *         原理图对应：
 *         PA1 / AF2 -> USART2_TX -> MAX485 DI
 *         PB1 / AF2 <- USART2_RX <- MAX485 RO
 *
 * @note   本测试只使用 USART2 发送。若 MAX485 的 DE 与 /RE 需要 MCU 控制，
 *         请在 USART2_StartTx() 前置发送方向，在 USART2 IRQ 的 TC 处恢复接收方向。
 ***********************************************************************************************************************/
static void USART2_RS485_Configure(uint32_t Baudrate)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    NVIC_InitTypeDef  NVIC_InitStruct;
    USART_InitTypeDef USART_InitStruct;

    RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1ENR_USART2, ENABLE);

    /* PA1：USART2_TX，AF2。 */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_2);
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PB1：USART2_RX，AF2。即使当前仅用于调试发送，也保留 RX 引脚配置。 */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_2);
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    USART_StructInit(&USART_InitStruct);
    USART_InitStruct.USART_BaudRate   = Baudrate;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits   = USART_StopBits_1;
    USART_InitStruct.USART_Parity     = USART_Parity_No;
    USART_InitStruct.USART_Mode       = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStruct);

    NVIC_InitStruct.NVIC_IRQChannel         = USART2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 0x01;
    NVIC_InitStruct.NVIC_IRQChannelCmd      = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    USART_Cmd(USART2, ENABLE);
}

/***********************************************************************************************************************
 * @brief  启动 USART1 的一帧 IDLE 接收
 *
 * @note   IDLE 代表接收线出现至少一个字符时间的空闲。
 *         它只表示本帧暂时结束，因此 TOF 帧最终仍需由帧头、长度、校验进一步确认。
 ***********************************************************************************************************************/
static void USART1_TOF_RxIdleStart(uint16_t MaxLength)
{
    uint16_t i;

    if (MaxLength > USART_RX_BUFFER_SIZE)
    {
        MaxLength = USART_RX_BUFFER_SIZE;
    }

    for (i = 0U; i < MaxLength; i++)
    {
        USART_RxStruct.Buffer[i] = 0U;
    }

    USART_RxStruct.Length       = MaxLength;
    USART_RxStruct.CurrentCount = 0U;
    USART_RxStruct.CompleteFlag = 0U;
    USART_RxStruct.OverflowFlag = 0U;

    /* 清除启动下一帧前遗留的接收状态。 */
    (void)USART_ReceiveData(USART1);
    USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    USART_ClearFlag(USART1, USART_FLAG_RXNE);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
}

/***********************************************************************************************************************
 * @brief  将一段 ASCII 数据通过 USART2 发送
 *
 * @note   数据先复制到全局发送缓冲区，再由 USART2_TXE 中断逐字节发送。
 ***********************************************************************************************************************/
static void USART2_StartTx(const uint8_t *Buffer, uint16_t Length)
{
    uint16_t i;

    if ((Buffer == 0) || (Length == 0U))
    {
        return;
    }

    if (Length > USART_RX_BUFFER_SIZE)
    {
        Length = USART_RX_BUFFER_SIZE;
    }

    /*
     * 当前测试要求每帧都输出。
     * 正常 27 字节 TOF 帧被转换为约 91 字节 ASCII；
     * 在 460800 bps 下约 2 ms 发完，远小于普通 TOF 读取周期。
     */
    while (USART_TxStruct.CompleteFlag == 0U)
    {
    }

    for (i = 0U; i < Length; i++)
    {
        USART_TxStruct.Buffer[i] = Buffer[i];
    }

    USART_TxStruct.Length       = Length;
    USART_TxStruct.CurrentCount = 0U;
    USART_TxStruct.CompleteFlag = 0U;

    USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
}

/***********************************************************************************************************************
 * @brief  将一帧 TOF 原始数据以十六进制文本形式打印到 RS485
 *
 * @example TOF RX: 5A 8E 17 37 29 ... 7D
 ***********************************************************************************************************************/
static void USART2_PrintTofHexFrame(const uint8_t *Frame, uint16_t Length)
{
    uint8_t  Text[RS485_DEBUG_TEXT_MAX_LENGTH];
    uint16_t i;
    uint16_t out;

    if ((Frame == 0) || (Length == 0U))
    {
        return;
    }

    if (Length > TOF_FRAME_MAX_LENGTH)
    {
        Length = TOF_FRAME_MAX_LENGTH;
    }

    out = 0U;

    Text[out++] = 'T';
    Text[out++] = 'O';
    Text[out++] = 'F';
    Text[out++] = ' ';
    Text[out++] = 'R';
    Text[out++] = 'X';
    Text[out++] = ':';
    Text[out++] = ' ';

    for (i = 0U; i < Length; i++)
    {
        Text[out++] = s_hex_table[(Frame[i] >> 4) & 0x0FU];
        Text[out++] = s_hex_table[ Frame[i]       & 0x0FU];
        Text[out++] = ' ';
    }

    Text[out++] = '\r';
    Text[out++] = '\n';

    USART2_StartTx(Text, out);
}

/***********************************************************************************************************************
 * @brief  双串口测试主循环
 *
 *          1. USART1 收到 TOF 一帧数据，IDLE 中断置 CompleteFlag；
 *          2. 主循环立即复制该帧，马上重开 USART1 接收；
 *          3. 将已冻结帧转换为 HEX 文本，由 USART2 发到 RS485；
 *          4. USART1 接收与 USART2 发送并行，不会因为字符打印阻塞 TOF 接收。
 ***********************************************************************************************************************/
void USART_Interrupt_Sample(void)
{
    uint8_t  FrameCopy[TOF_FRAME_MAX_LENGTH];
    uint16_t FrameLength;
    uint16_t i;
    uint8_t  Overflow;

    USART_RxStruct.CompleteFlag = 0U;
    USART_TxStruct.CompleteFlag = 1U;

    USART1_TOF_Configure(TOF_USART_BAUDRATE);
    USART2_RS485_Configure(RS485_DEBUG_BAUDRATE);

    USART1_TOF_RxIdleStart(TOF_FRAME_MAX_LENGTH);

    while (1)
    {
        if (USART_RxStruct.CompleteFlag != 0U)
        {
            /*
             * USART1 在 IDLE ISR 中已关闭 RXNE/IDLE，
             * 因而下面复制 Buffer 的期间，数据不会被 ISR 改写。
             */
            FrameLength = USART_RxStruct.CurrentCount;
            Overflow    = USART_RxStruct.OverflowFlag;

            if (FrameLength > TOF_FRAME_MAX_LENGTH)
            {
                FrameLength = TOF_FRAME_MAX_LENGTH;
                Overflow = 1U;
            }

            for (i = 0U; i < FrameLength; i++)
            {
                FrameCopy[i] = USART_RxStruct.Buffer[i];
            }

            /*
             * 必须先重新打开 TOF 接收，再开始 RS485 文本发送。
             * 这样 TX 期间，下一帧 TOF 数据仍可进入 USART1。
             */
            USART1_TOF_RxIdleStart(TOF_FRAME_MAX_LENGTH);

            if ((Overflow == 0U) && (FrameLength > 0U))
            {
                USART2_PrintTofHexFrame(FrameCopy, FrameLength);
            }
        }
    }
}
