/**
 * @file usart_interrupt.c
 * @brief YC02 TOF USART1 中断收帧、解析和 USART2 非阻塞 RS485 调试输出。
 *
 * USART1：PA12 / AF1 = TX，PA3 / AF1 = RX，460800bps。
 * USART2：PA1 / AF2 = TX，PB1 / AF2 = RX，PB0 = MAX485 T/R。
 */
#define _USART_INTERRUPT_C_

#include "platform.h"
#include "usart_interrupt.h"

#define RS485_DIR_PORT                  GPIOB
#define RS485_DIR_PIN                   GPIO_Pin_0
#define DEBUG_TX_MASK                   (USART2_DEBUG_TX_BUFFER_SIZE - 1U)

volatile USART_RxTx_TypeDef USART_RxStruct = {0};
volatile int16_t g_tof_distance_mm = 0;
volatile uint8_t g_tof_confidence = 0U;
volatile uint8_t g_tof_frame_ok = 0U;
volatile uint16_t g_tof_last_frame_length = 0U;
volatile uint32_t g_tof_valid_frame_count = 0U;
volatile uint32_t g_tof_bad_frame_count = 0U;
volatile uint32_t g_tof_dropped_frame_count = 0U;
volatile uint32_t g_tof_last_valid_tick_ms = 0U;
volatile uint32_t g_usart2_debug_drop_count = 0U;

/*
 * TOF 主循环只消费一份待处理帧。若主循环异常卡死，新帧不会覆盖旧帧，
 * 而是计入 dropped，避免半帧和数据竞争。
 */
static volatile TOF_Frame_t g_tof_pending_frame = {0};
static volatile uint8_t g_tof_pending_ready = 0U;

/* USART2 TX 环形队列：ISR 仅移动 tail，主循环仅移动 head。 */
static volatile uint8_t g_debug_tx_buf[USART2_DEBUG_TX_BUFFER_SIZE] = {0};
static volatile uint8_t g_debug_tx_head = 0U;
static volatile uint8_t g_debug_tx_tail = 0U;
static volatile uint8_t g_debug_tx_tc_wait = 0U;

static uint16_t read_le16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t read_le24(const uint8_t *p)
{
    return (uint32_t)((uint32_t)p[0] |
                      ((uint32_t)p[1] << 8) |
                      ((uint32_t)p[2] << 16));
}

static int32_t read_le32s(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] |
                     ((uint32_t)p[1] << 8) |
                     ((uint32_t)p[2] << 16) |
                     ((uint32_t)p[3] << 24));
}

static uint8_t yc02_checksum(const uint8_t *buf, uint16_t len_without_checksum)
{
    uint16_t i;
    uint32_t sum;

    sum = 0U;

    for (i = 0U; i < len_without_checksum; i++)
    {
        sum += buf[i];
    }

    return (uint8_t)(sum & 0xFFU);
}

static uint8_t YC02_ParseFrame(const uint8_t *buf,
                               uint16_t len,
                               TOF_Frame_t *out)
{
    if ((buf == 0) || (out == 0))
    {
        return 0U;
    }

    if (len != TOF_FRAME_LENGTH)
    {
        return 0U;
    }

    if ((buf[0] != 0x5AU) || (buf[1] != 0x8EU) || (buf[2] != 0x17U))
    {
        return 0U;
    }

    if (yc02_checksum(buf, TOF_FRAME_LENGTH - 1U) != buf[TOF_FRAME_LENGTH - 1U])
    {
        return 0U;
    }

    out->norm_tof = read_le16(&buf[3]);
    out->norm_peak = read_le16(&buf[5]);
    out->norm_noise = read_le24(&buf[7]);
    out->multshot = read_le16(&buf[10]);
    out->atten_peak = read_le16(&buf[12]);
    out->atten_noise = read_le24(&buf[14]);
    out->ref_tof = read_le16(&buf[17]);
    out->temperature_centi = read_le32s(&buf[19]);
    out->distance_mm = (int16_t)read_le16(&buf[23]);
    out->confidence = buf[25];
    out->tick_ms = PLATFORM_GetTickMs();

    return 1U;
}

/** @brief 配置 USART1 为 TOF 串口。 */
void USART_Configure(uint32_t Baudrate)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
    USART_InitTypeDef USART_InitStruct;

    NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 0x00U;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_GPIOA, ENABLE);

    /* PA12 / AF1 -> USART1_TX。 */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_1);
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PA3 / AF1 -> USART1_RX。 */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    RCC_APB1PeriphClockCmd(RCC_APB1ENR_USART1, ENABLE);

    USART_StructInit(&USART_InitStruct);
    USART_InitStruct.USART_BaudRate = Baudrate;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStruct);

    USART_ITConfig(USART1, USART_IT_PE, ENABLE);
    USART_ITConfig(USART1, USART_IT_ERR, ENABLE);
    USART_Cmd(USART1, ENABLE);
}

static void USART2_DebugConfigure(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
    USART_InitTypeDef USART_InitStruct;

    RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_GPIOB, ENABLE);

    /* PA1 / AF2 -> USART2_TX -> MAX485 DI。 */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_2);
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PB1 / AF2 -> USART2_RX <- MAX485 RO，预留给后续 RS485 指令。 */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_2);
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* PB0：MAX485 DE 和 /RE 并联的 T/R，默认接收且驱动关闭。 */
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin = RS485_DIR_PIN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(RS485_DIR_PORT, &GPIO_InitStruct);
    GPIO_WriteBit(RS485_DIR_PORT, RS485_DIR_PIN, Bit_RESET);

    RCC_APB1PeriphClockCmd(RCC_APB1ENR_USART2, ENABLE);

    USART_StructInit(&USART_InitStruct);
    USART_InitStruct.USART_BaudRate = USART2_DEBUG_BAUDRATE;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStruct);

    USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
    USART_ITConfig(USART2, USART_IT_TC, DISABLE);

    NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 0x02U;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    USART_Cmd(USART2, ENABLE);
}

void USART_RxIdle_Start(uint16_t MaxLength)
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

    USART_RxStruct.Length = MaxLength;
    USART_RxStruct.CurrentCount = 0U;
    USART_RxStruct.CompleteFlag = 0U;
    USART_RxStruct.OverflowFlag = 0U;

    /* 清掉历史 RXNE/IDLE 状态后重新打开收帧。 */
    (void)USART_ReceiveData(USART1);
    USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    USART_ClearFlag(USART1, USART_FLAG_RXNE);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
}

void TOF_CommInit(uint32_t Baudrate)
{
    g_tof_pending_ready = 0U;
    g_tof_frame_ok = 0U;
    g_tof_valid_frame_count = 0U;
    g_tof_bad_frame_count = 0U;
    g_tof_dropped_frame_count = 0U;
    g_tof_last_valid_tick_ms = PLATFORM_GetTickMs();

    USART_Configure(Baudrate);
    USART2_DebugConfigure();
    USART_RxIdle_Start(TOF_RX_MAX_LENGTH);
}

/**
 * @brief 必须在 main while(1) 内高频调用。
 *        该函数只做一次短帧解析和立即重启 RX，不做打印、不做距离判断。
 */
void TOF_Service(void)
{
    TOF_Frame_t frame;
    uint16_t count;

    if (USART_RxStruct.CompleteFlag == 0U)
    {
        return;
    }

    count = USART_RxStruct.CurrentCount;
    g_tof_last_frame_length = count;

    if ((USART_RxStruct.OverflowFlag == 0U) &&
        (YC02_ParseFrame((const uint8_t *)USART_RxStruct.Buffer, count, &frame) != 0U))
    {
        g_tof_distance_mm = frame.distance_mm;
        g_tof_confidence = frame.confidence;
        g_tof_frame_ok = 1U;
        g_tof_valid_frame_count++;
        g_tof_last_valid_tick_ms = frame.tick_ms;

        if (g_tof_pending_ready == 0U)
        {
            g_tof_pending_frame = frame;
            g_tof_pending_ready = 1U;
        }
        else
        {
            /* 主循环未及时消费上一帧，宁可丢弃新帧，也绝不覆盖待判定数据。 */
            g_tof_dropped_frame_count++;
        }
    }
    else
    {
        g_tof_frame_ok = 0U;
        g_tof_bad_frame_count++;
    }

    /* 必须尽快重新打开 RXNE/IDLE，故置于所有业务处理之前。 */
    USART_RxIdle_Start(TOF_RX_MAX_LENGTH);
}

uint8_t TOF_PopValidFrame(TOF_Frame_t *frame)
{
    if ((frame == 0) || (g_tof_pending_ready == 0U))
    {
        return 0U;
    }

    /* 写入 g_tof_pending_frame 的唯一上下文是主循环 TOF_Service，无需关中断。 */
    *frame = g_tof_pending_frame;
    g_tof_pending_ready = 0U;

    return 1U;
}

/**
 * @brief USART2 非阻塞入队。
 *        队列满时丢弃新字节并计数；绝不阻塞 TOF 收帧和距离判定。
 */
void USART2_DebugWriteByte(uint8_t data)
{
    uint8_t head;
    uint8_t next;

    head = g_debug_tx_head;
    next = (uint8_t)((head + 1U) & DEBUG_TX_MASK);

    if (next == g_debug_tx_tail)
    {
        g_usart2_debug_drop_count++;
        return;
    }

    g_debug_tx_buf[head] = data;
    g_debug_tx_head = next;

    /* 先打开 485 发送使能，再开 TXE。 */
    GPIO_WriteBit(RS485_DIR_PORT, RS485_DIR_PIN, Bit_SET);
    g_debug_tx_tc_wait = 0U;
    USART_ITConfig(USART2, USART_IT_TC, DISABLE);
    USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
}

void USART2_DebugWriteString(const char *str)
{
    if (str == 0)
    {
        return;
    }

    while (*str != '\0')
    {
        USART2_DebugWriteByte((uint8_t)*str);
        str++;
    }
}

void USART2_DebugWriteU32(uint32_t value)
{
    char buf[10];
    uint8_t count;

    count = 0U;

    if (value == 0U)
    {
        USART2_DebugWriteByte((uint8_t)'0');
        return;
    }

    while ((value != 0U) && (count < sizeof(buf)))
    {
        buf[count++] = (char)('0' + (value % 10U));
        value /= 10U;
    }

    while (count != 0U)
    {
        count--;
        USART2_DebugWriteByte((uint8_t)buf[count]);
    }
}

void USART2_DebugWriteU16(uint16_t value)
{
    USART2_DebugWriteU32((uint32_t)value);
}

void USART2_DebugWriteS16(int16_t value)
{
    if (value < 0)
    {
        USART2_DebugWriteByte((uint8_t)'-');
        USART2_DebugWriteU32((uint32_t)(-(int32_t)value));
    }
    else
    {
        USART2_DebugWriteU32((uint32_t)value);
    }
}

void USART2_DebugWriteTemperature(int32_t temperature_centi)
{
    uint32_t abs_value;

    if (temperature_centi < 0)
    {
        USART2_DebugWriteByte((uint8_t)'-');
        abs_value = (uint32_t)(-(temperature_centi + 1));
        abs_value += 1U;
    }
    else
    {
        abs_value = (uint32_t)temperature_centi;
    }

    USART2_DebugWriteU32(abs_value / 100U);
    USART2_DebugWriteByte((uint8_t)'.');
    USART2_DebugWriteByte((uint8_t)('0' + ((abs_value / 10U) % 10U)));
    USART2_DebugWriteByte((uint8_t)('0' + (abs_value % 10U)));
    USART2_DebugWriteString(" C");
}

/** @brief 由 USART2_IRQHandler 调用。 */
void USART2_DebugTxIRQHandler(void)
{
    uint8_t tail;

    if (USART_GetITStatus(USART2, USART_IT_TXE) != RESET)
    {
        tail = g_debug_tx_tail;

        if (tail != g_debug_tx_head)
        {
            USART_SendData(USART2, g_debug_tx_buf[tail]);
            g_debug_tx_tail = (uint8_t)((tail + 1U) & DEBUG_TX_MASK);
        }
        else
        {
            /* 缓冲已空，但最后一个字节仍可能在移位寄存器中。 */
            USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
            g_debug_tx_tc_wait = 1U;
            USART_ITConfig(USART2, USART_IT_TC, ENABLE);
        }
    }

    if (USART_GetITStatus(USART2, USART_IT_TC) != RESET)
    {
        if ((g_debug_tx_tc_wait != 0U) && (g_debug_tx_tail == g_debug_tx_head))
        {
            USART_ITConfig(USART2, USART_IT_TC, DISABLE);
            g_debug_tx_tc_wait = 0U;
            GPIO_WriteBit(RS485_DIR_PORT, RS485_DIR_PIN, Bit_RESET);
        }
    }
}
