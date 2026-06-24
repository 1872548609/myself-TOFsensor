/**
 * @file usart_interrupt.c
 * @brief YC02 TOF 的 USART1 中断接收测试。
 *
 * 硬件对应关系：
 * PA12 / AF1 -> USART1_TX -> TOF RXD
 * PA3  / AF1 -> USART1_RX <- TOF TXD
 */

#define _USART_INTERRUPT_C_

#include "platform.h"
#include "usart_interrupt.h"

volatile USART_RxTx_TypeDef USART_RxStruct = {0};
volatile USART_RxTx_TypeDef USART_TxStruct = {0};

volatile int16_t  g_tof_distance_mm = 0;
volatile uint8_t  g_tof_confidence = 0;
volatile uint8_t  g_tof_frame_ok = 0;
volatile uint16_t g_tof_last_frame_length = 0;
volatile uint32_t g_tof_valid_frame_count = 0;
volatile uint32_t g_tof_bad_frame_count = 0;

typedef struct
{
    uint16_t norm_tof;
    uint16_t norm_peak;
    uint32_t norm_noise;
    uint16_t multshot;
    uint16_t atten_peak;
    uint32_t atten_noise;
    uint16_t ref_tof;
    int32_t  temperature;     /* 0.01 摄氏度 */
    int16_t  cal_tof;         /* mm */
    uint8_t  confidence;
} YC02_FrameData_t;

static uint16_t read_le16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t read_le24(const uint8_t *p)
{
    return (uint32_t)((uint32_t)p[0]
                    | ((uint32_t)p[1] << 8)
                    | ((uint32_t)p[2] << 16));
}

static int32_t read_le32s(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0]
                   | ((uint32_t)p[1] << 8)
                   | ((uint32_t)p[2] << 16)
                   | ((uint32_t)p[3] << 24));
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
                               YC02_FrameData_t *out)
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

    out->norm_tof    = read_le16(&buf[3]);
    out->norm_peak   = read_le16(&buf[5]);
    out->norm_noise  = read_le24(&buf[7]);
    out->multshot    = read_le16(&buf[10]);
    out->atten_peak  = read_le16(&buf[12]);
    out->atten_noise = read_le24(&buf[14]);
    out->ref_tof     = read_le16(&buf[17]);
    out->temperature = read_le32s(&buf[19]);
    out->cal_tof     = (int16_t)read_le16(&buf[23]);
    out->confidence  = buf[25];

    return 1U;
}

void USART_Configure(uint32_t Baudrate)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    NVIC_InitTypeDef  NVIC_InitStruct;
    USART_InitTypeDef USART_InitStruct;

    /* USART1 中断：TOF 专用。 */
    NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 0x01U;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_GPIOA, ENABLE);

    /* PA12 = USART1_TX，AF1，接 TOF RXD。 */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_1);
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PA3 = USART1_RX，AF1，接 TOF TXD。 */
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

    /* 清掉上一次接收残留状态后，再打开 RXNE/IDLE 中断。 */
    (void)USART_ReceiveData(USART1);
    USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    USART_ClearFlag(USART1, USART_FLAG_RXNE);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
}

void USART_Interrupt_Sample(void)
{
    YC02_FrameData_t frame;

    USART_Configure(460800U);

    USART_RxStruct.CompleteFlag = 0U;
    USART_TxStruct.CompleteFlag = 1U;

    /*
     * 保持原测试代码的 IDLE 结束方式。
     * 27 字节帧使用 30 字节上限，以便检测异常超长帧。
     */
    USART_RxIdle_Start(TOF_RX_MAX_LENGTH);

    PLATFORM_LED_Enable(LED1, DISABLE);
    PLATFORM_LED_Enable(LED2, DISABLE);

    while (1)
    {
        if (USART_RxStruct.CompleteFlag != 0U)
        {
            g_tof_last_frame_length = USART_RxStruct.CurrentCount;

            if ((USART_RxStruct.OverflowFlag == 0U)
             && (YC02_ParseFrame((const uint8_t *)USART_RxStruct.Buffer,
                                 USART_RxStruct.CurrentCount,
                                 &frame) != 0U))
            {
                g_tof_distance_mm = frame.cal_tof;
                g_tof_confidence = frame.confidence;
                g_tof_frame_ok = 1U;
                g_tof_valid_frame_count++;

                /* 有效帧：绿灯亮、红灯灭。 */
                PLATFORM_LED_Enable(LED1, ENABLE);
                PLATFORM_LED_Enable(LED2, DISABLE);
            }
            else
            {
                g_tof_frame_ok = 0U;
                g_tof_bad_frame_count++;

                /* 无效帧：红灯亮、绿灯灭。 */
                PLATFORM_LED_Enable(LED1, DISABLE);
                PLATFORM_LED_Enable(LED2, ENABLE);
            }

            /* 尽快重启下一帧接收，避免因调试打印产生丢帧。 */
            USART_RxIdle_Start(TOF_RX_MAX_LENGTH);
        }
    }
}
