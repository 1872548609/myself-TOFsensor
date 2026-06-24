/**
 * @file usart_interrupt.c
 * @brief YC02 TOF USART1 接收 + USART2 按键回传测试。
 *
 * TOF：PA12 / AF1 -> USART1_TX，PA3 / AF1 -> USART1_RX。
 * 调试：PA1 / AF2 -> USART2_TX，经 MAX485 DI 输出至 485_A / 485_B。
 *       PB1 / AF2 -> USART2_RX（当前不使用，仅保留配置）。
 *       PB0       -> RS485 T/R 方向控制（需要 PCB 实际已将 PB0 接至 T/R）。
 * 按键：PA7（KEY_BLUE）按下时，回传最近一帧校验通过后的解析数据。
 */

#define _USART_INTERRUPT_C_

#include "platform.h"
#include "usart_interrupt.h"

/* USART1 TOF 接收缓冲。 */
volatile USART_RxTx_TypeDef USART_RxStruct = {0};
volatile USART_RxTx_TypeDef USART_TxStruct = {0};

volatile int16_t  g_tof_distance_mm = 0;
volatile uint8_t  g_tof_confidence = 0;
volatile uint8_t  g_tof_frame_ok = 0;
volatile uint16_t g_tof_last_frame_length = 0;
volatile uint32_t g_tof_valid_frame_count = 0;
volatile uint32_t g_tof_bad_frame_count = 0;

volatile uint8_t  g_tof_last_frame[TOF_RX_MAX_LENGTH] = {0};
volatile uint16_t g_tof_last_frame_count = 0;
volatile uint8_t  g_tof_last_frame_overflow = 0;

typedef struct
{
    uint16_t norm_tof;
    uint16_t norm_peak;
    uint32_t norm_noise;
    uint16_t multshot;
    uint16_t atten_peak;
    uint32_t atten_noise;
    uint16_t ref_tof;
    int32_t  temperature;
    int16_t  cal_tof;
    uint8_t  confidence;
} YC02_FrameData_t;

/*
 * 最近一次通过帧头、长度和校验验证的解析结果。
 * 仅在主循环中更新、仅在按键打印时读取，因此不需要额外临界区。
 */
static YC02_FrameData_t g_tof_last_parsed = {0};
static uint8_t g_tof_last_parsed_valid = 0U;

/* 当前硬件：蓝键 PA7 按下为低电平。 */
#define TOF_PRINT_KEY_PORT        GPIOA
#define TOF_PRINT_KEY_PIN         GPIO_Pin_7

/*
 * MAX485 的 /RE 和 DE 并联为 T/R：
 * T/R=1 -> 发送；T/R=0 -> 接收/驱动关闭。
 * 原理图中 PB0 只有未标注连线，请先确认 PCB 上 PB0 的确连到 T/R。
 */
#define RS485_DIR_PORT             GPIOB
#define RS485_DIR_PIN              GPIO_Pin_0

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
    uint32_t sum = 0U;

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

/**
 * @brief 配置 USART1 为 TOF 收发口。
 */
void USART_Configure(uint32_t Baudrate)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    NVIC_InitTypeDef  NVIC_InitStruct;
    USART_InitTypeDef USART_InitStruct;

    NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 0x01U;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_GPIOA, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_1);
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

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

/**
 * @brief 配置 USART2，仅用于通过板载 RS485 向电脑发送调试文本。
 */
static void USART2_DebugConfigure(void)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;

    RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_GPIOB, ENABLE);

    /* PA1 / AF2 = USART2_TX，接 MAX485 的 DI。 */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_2);
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PB1 / AF2 = USART2_RX，接 MAX485 的 RO；当前调试不读取。 */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_2);
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* PB0 假定为 MAX485 T/R：默认接收/发送器关闭，避免上电占线。 */
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
    USART_Cmd(USART2, ENABLE);
}

static void USART2_DebugBeginTx(void)
{
    GPIO_WriteBit(RS485_DIR_PORT, RS485_DIR_PIN, Bit_SET);
}

static void USART2_DebugEndTx(void)
{
    while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
    {
    }

    GPIO_WriteBit(RS485_DIR_PORT, RS485_DIR_PIN, Bit_RESET);
}

static void USART2_DebugWriteByte(uint8_t data)
{
    USART_SendData(USART2, data);

    while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
    {
    }
}

static void USART2_DebugWriteString(const char *str)
{
    while (*str != '\0')
    {
        USART2_DebugWriteByte((uint8_t)*str);
        str++;
    }
}

static void USART2_DebugWriteU32(uint32_t value)
{
    char buf[10];
    uint8_t count = 0U;

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

static void USART2_DebugWriteU16(uint16_t value)
{
    USART2_DebugWriteU32((uint32_t)value);
}

static void USART2_DebugWriteS16(int16_t value)
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

/**
 * @brief 按 0.01 摄氏度的原始单位输出温度，例如 4011 -> 40.11 C。
 */
static void USART2_DebugWriteTemperature(int32_t temperature_centi)
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

/**
 * @brief 蓝键按下时，通过 USART2 回传最近一帧的解析结果。
 * @note 发送期间会短暂阻塞主循环，属于测试用途；不要用于最终连续通信版本。
 */
static void USART2_DebugPrintLastFrame(void)
{
    USART2_DebugBeginTx();

    if (g_tof_last_parsed_valid == 0U)
    {
        USART2_DebugWriteString("\r\n[TOF] No valid parsed frame\r\n");
        USART2_DebugWriteString("LEN = ");
        USART2_DebugWriteU16(g_tof_last_frame_count);
        USART2_DebugWriteString("\r\n");
        USART2_DebugEndTx();
        return;
    }

    USART2_DebugWriteString("\r\nnorm_tof    = ");
    USART2_DebugWriteU16(g_tof_last_parsed.norm_tof);

    USART2_DebugWriteString("\r\nnorm_peak   = ");
    USART2_DebugWriteU16(g_tof_last_parsed.norm_peak);

    USART2_DebugWriteString("\r\nnorm_noise  = ");
    USART2_DebugWriteU32(g_tof_last_parsed.norm_noise);

    USART2_DebugWriteString("\r\nmultshot    = ");
    USART2_DebugWriteU16(g_tof_last_parsed.multshot);

    USART2_DebugWriteString("\r\natten_peak  = ");
    USART2_DebugWriteU16(g_tof_last_parsed.atten_peak);

    USART2_DebugWriteString("\r\natten_noise = ");
    USART2_DebugWriteU32(g_tof_last_parsed.atten_noise);

    USART2_DebugWriteString("\r\nref_tof     = ");
    USART2_DebugWriteU16(g_tof_last_parsed.ref_tof);

    USART2_DebugWriteString("\r\ntemperature = ");
    USART2_DebugWriteTemperature(g_tof_last_parsed.temperature);

    USART2_DebugWriteString("\r\ncal_tof     = ");
    USART2_DebugWriteS16(g_tof_last_parsed.cal_tof);
    USART2_DebugWriteString(" mm");

    USART2_DebugWriteString("\r\nconfidence  = ");
    USART2_DebugWriteU16((uint16_t)g_tof_last_parsed.confidence);
    USART2_DebugWriteString("\r\n");

    USART2_DebugEndTx();
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

    (void)USART_ReceiveData(USART1);
    USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    USART_ClearFlag(USART1, USART_FLAG_RXNE);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
}

/**
 * @brief 保存刚完成的一帧，无论其最终校验成功与否。
 */
static void TOF_SaveLastFrame(void)
{
    uint16_t i;
    uint16_t count = USART_RxStruct.CurrentCount;

    if (count > TOF_RX_MAX_LENGTH)
    {
        count = TOF_RX_MAX_LENGTH;
    }

    for (i = 0U; i < count; i++)
    {
        g_tof_last_frame[i] = USART_RxStruct.Buffer[i];
    }

    g_tof_last_frame_count = count;
    g_tof_last_frame_overflow = USART_RxStruct.OverflowFlag;
}

void USART_Interrupt_Sample(void)
{
    YC02_FrameData_t frame;
    uint8_t key_latched = 0U;

    USART_Configure(460800U);
    USART2_DebugConfigure();

    USART_RxStruct.CompleteFlag = 0U;
    USART_TxStruct.CompleteFlag = 1U;

    USART_RxIdle_Start(TOF_RX_MAX_LENGTH);

    PLATFORM_LED_Enable(LED1, DISABLE);
    PLATFORM_LED_Enable(LED2, DISABLE);

    while (1)
    {
        if (USART_RxStruct.CompleteFlag != 0U)
        {
            g_tof_last_frame_length = USART_RxStruct.CurrentCount;
            TOF_SaveLastFrame();

            if ((USART_RxStruct.OverflowFlag == 0U)
             && (YC02_ParseFrame((const uint8_t *)USART_RxStruct.Buffer,
                                 USART_RxStruct.CurrentCount,
                                 &frame) != 0U))
            {
                g_tof_last_parsed = frame;
                g_tof_last_parsed_valid = 1U;

                g_tof_distance_mm = frame.cal_tof;
                g_tof_confidence = frame.confidence;
                g_tof_frame_ok = 1U;
                g_tof_valid_frame_count++;

                PLATFORM_LED_Enable(LED1, ENABLE);
                PLATFORM_LED_Enable(LED2, DISABLE);
            }
            else
            {
                g_tof_last_parsed_valid = 0U;
                g_tof_frame_ok = 0U;
                g_tof_bad_frame_count++;

                PLATFORM_LED_Enable(LED1, DISABLE);
                PLATFORM_LED_Enable(LED2, ENABLE);
            }

            USART_RxIdle_Start(TOF_RX_MAX_LENGTH);
        }

        /*
         * 低电平为按下；使用锁存保证长按仅回传一次。
         * 释放后才允许下一次触发。
         */
        if (GPIO_ReadInputDataBit(TOF_PRINT_KEY_PORT, TOF_PRINT_KEY_PIN) == Bit_RESET)
        {
            if (key_latched == 0U)
            {
                key_latched = 1U;
                USART2_DebugPrintLastFrame();
            }
        }
        else
        {
            key_latched = 0U;
        }
    }
}
