/**
 * @file usart_interrupt.c
 * @brief
 * YC02 TOF 模块的 USART1 中断收帧、帧解析，
 * 以及 USART2 + MAX485 的非阻塞 RS485 调试发送。
 *
 * 硬件连接：
 *
 * USART1：连接 TOF 模块
 *   PA12 / AF1 = USART1_TX -> TOF_RXD
 *   PA3  / AF1 = USART1_RX <- TOF_TXD
 *   波特率：460800bps
 *
 * USART2：连接 MAX485，用于电脑调试输出或后续 RS485 通讯
 *   PA1 / AF2 = USART2_TX -> MAX485_DI
 *   PB1 / AF2 = USART2_RX <- MAX485_RO
 *   PB0       = MAX485_DE 和 /RE，共用收发方向控制
 */
#define _USART_INTERRUPT_C_

#include "platform.h"
#include "usart_interrupt.h"

/* ----------------------------- RS485 硬件定义 ----------------------------- */

/* MAX485 的发送/接收方向控制脚所在端口。 */
#define RS485_DIR_PORT                  GPIOB

/* PB0 同时连接 MAX485 的 DE 和 /RE：
 * PB0 = 1：发送使能，接收关闭；
 * PB0 = 0：发送关闭，接收使能。
 */
#define RS485_DIR_PIN                   GPIO_Pin_0

/* USART2 软件发送环形缓冲区掩码。
 *
 * USART2_DEBUG_TX_BUFFER_SIZE 必须为 2 的整数次幂，
 * 例如 64、128、256。
 *
 * 例如缓冲区长度为 128：
 *   下标范围为 0~127；
 *   使用 & 127 可实现自动回绕。
 */
#define DEBUG_TX_MASK                   (USART2_DEBUG_TX_BUFFER_SIZE - 1U)


/* ----------------------------- 全局通讯状态 ----------------------------- */

/* USART1 接收控制结构体。
 *
 * 内部通常包含：
 * Buffer[]       ：接收缓冲区；
 * Length         ：允许接收的最大长度；
 * CurrentCount   ：当前已经收到的字节数；
 * CompleteFlag   ：检测到 IDLE 后，表示一帧接收完成；
 * OverflowFlag   ：接收字节数超过最大长度时置位。
 *
 * volatile：
 * 该变量会在 USART1 中断和主循环之间共同访问，
 * 防止编译器优化导致主循环读不到中断更新后的最新值。
 */
volatile USART_RxTx_TypeDef USART_RxStruct = {0};

/* 最近一次校验通过的 TOF 距离值，单位 mm。 */
volatile int16_t g_tof_distance_mm = 0;

/* 最近一次校验通过的置信度。 */
volatile uint8_t g_tof_confidence = 0U;

/* 最近一次处理的帧是否有效。
 * 1：上一帧格式、帧头、校验和均正确；
 * 0：上一帧无效。
 */
volatile uint8_t g_tof_frame_ok = 0U;

/* 最近一次接收完成时的实际帧长度。 */
volatile uint16_t g_tof_last_frame_length = 0U;

/* 累计接收到的有效 TOF 帧数量。 */
volatile uint32_t g_tof_valid_frame_count = 0U;

/* 累计接收到的无效 TOF 帧数量。 */
volatile uint32_t g_tof_bad_frame_count = 0U;

/* 累计丢弃的新帧数量。
 *
 * 当上一帧还没有被主业务逻辑消费时，
 * 为防止新帧覆盖旧帧，此处丢弃新帧并计数。
 */
volatile uint32_t g_tof_dropped_frame_count = 0U;

/* 最近一次收到有效 TOF 帧的系统毫秒时间戳。 */
volatile uint32_t g_tof_last_valid_tick_ms = 0U;

/* USART2 调试发送环形队列满时，被丢弃的字节数量。 */
volatile uint32_t g_usart2_debug_drop_count = 0U;


/* ----------------------- TOF 待消费帧的单帧邮箱 ------------------------ */

/*
 * TOF 主循环只保留一份“待处理帧”。
 *
 * 设计目的：
 * 1. USART1 中断只负责收字节，不做耗时解析；
 * 2. TOF_Service() 在主循环中完成帧解析；
 * 3. 主业务逻辑通过 TOF_PopValidFrame() 取走完整帧；
 * 4. 若主业务逻辑没有及时取走上一帧，新帧直接丢弃；
 * 5. 不允许覆盖旧帧，避免主业务读到“前后拼接”的错误数据。
 */

/* 已解析、等待主业务取走的完整 TOF 帧。 */
static volatile TOF_Frame_t g_tof_pending_frame = {0};

/* 待处理帧状态标志：
 * 0：当前没有待处理帧；
 * 1：g_tof_pending_frame 内保存了一帧有效数据。
 */
static volatile uint8_t g_tof_pending_ready = 0U;


/* ------------------------ USART2 调试发送环形队列 ----------------------- */

/*
 * USART2 的发送采用“软件环形缓冲区 + TXE 中断”方式。
 *
 * 主循环：
 *   负责把调试字符写入 g_debug_tx_buf[]，
 *   只推进 g_debug_tx_head。
 *
 * USART2 中断：
 *   负责从 g_debug_tx_buf[] 取字符并写入硬件发送寄存器，
 *   只推进 g_debug_tx_tail。
 *
 * 这样可以避免主循环等待串口逐字节发送，
 * 防止调试打印影响 TOF 收帧和距离判定实时性。
 */

/* USART2 调试输出的软件发送缓冲区。 */
static volatile uint8_t g_debug_tx_buf[USART2_DEBUG_TX_BUFFER_SIZE] = {0};

/* 环形队列写指针：主循环写入字符后推进。 */
static volatile uint8_t g_debug_tx_head = 0U;

/* 环形队列读指针：USART2 TXE 中断发送字符后推进。 */
static volatile uint8_t g_debug_tx_tail = 0U;

/* 发送完成等待标志。
 *
 * 0：当前不等待最后一个字节发完；
 * 1：软件缓冲已空，但最后一个字节可能还在 USART 移位寄存器中，
 *    此时需要等待 TC（Transmission Complete）中断。
 */
static volatile uint8_t g_debug_tx_tc_wait = 0U;


/* -------------------------- 小端数据转换函数 --------------------------- */

/**
 * @brief 从字节数组读取一个 16 位小端无符号整数。
 *
 * 小端格式：
 *   p[0]：低字节
 *   p[1]：高字节
 *
 * 例如：
 *   p[0] = 0x34
 *   p[1] = 0x12
 * 返回：
 *   0x1234
 *
 * @param p 指向至少两个字节的数据地址。
 * @return 16 位无符号整数。
 */
static uint16_t read_le16(const uint8_t *p)
{
    /* p[0] 作为低 8 位；
     * p[1] 左移 8 位后作为高 8 位；
     * 两者按位或组合为完整 16 位数据。
     */
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}


/**
 * @brief 从字节数组读取一个 24 位小端无符号整数。
 *
 * 小端格式：
 *   p[0]：bit0~7
 *   p[1]：bit8~15
 *   p[2]：bit16~23
 *
 * @param p 指向至少三个字节的数据地址。
 * @return 32 位变量承载的 24 位无符号数据。
 */
static uint32_t read_le24(const uint8_t *p)
{
    return (uint32_t)((uint32_t)p[0] |
                      ((uint32_t)p[1] << 8) |
                      ((uint32_t)p[2] << 16));
}


/**
 * @brief 从字节数组读取一个 32 位小端有符号整数。
 *
 * @param p 指向至少四个字节的数据地址。
 * @return 32 位有符号整数。
 */
static int32_t read_le32s(const uint8_t *p)
{
    /* 先将四个字节拼接为 uint32_t，
     * 再强制转换成 int32_t。
     *
     * 若最高位 bit31 为 1，转换后自动表示负数补码。
     */
    return (int32_t)((uint32_t)p[0] |
                     ((uint32_t)p[1] << 8) |
                     ((uint32_t)p[2] << 16) |
                     ((uint32_t)p[3] << 24));
}


/* ----------------------------- YC02 帧校验 ----------------------------- */

/**
 * @brief 计算 YC02 帧的 8 位累加和校验值。
 *
 * 校验规则：
 *   将从帧头开始到校验字节前的所有字节累加，
 *   仅保留累加结果的低 8 位。
 *
 * @param buf                  指向待校验帧数据。
 * @param len_without_checksum 不包含最后一个校验字节的长度。
 * @return 校验和低 8 位。
 */
static uint8_t yc02_checksum(const uint8_t *buf, uint16_t len_without_checksum)
{
    uint16_t i;
    uint32_t sum;

    /* 用 32 位变量累计，避免短帧相加时发生中间溢出。 */
    sum = 0U;

    /* 逐字节累加。 */
    for (i = 0U; i < len_without_checksum; i++)
    {
        sum += buf[i];
    }

    /* 仅取低 8 位作为最终校验和。 */
    return (uint8_t)(sum & 0xFFU);
}


/**
 * @brief 校验并解析一帧 YC02 TOF 数据。
 *
 * 处理步骤：
 * 1. 检查输入指针；
 * 2. 检查帧长；
 * 3. 检查帧头；
 * 4. 检查累加和；
 * 5. 按 YC02 协议解析各字段；
 * 6. 记录该帧被解析时的系统时间。
 *
 * @param buf 原始接收字节缓冲区。
 * @param len 实际接收到的字节数量。
 * @param out 用于保存解析结果的 TOF_Frame_t 结构体。
 *
 * @return
 * 1：帧有效且解析成功；
 * 0：帧无效或参数错误。
 */
static uint8_t YC02_ParseFrame(const uint8_t *buf,
                               uint16_t len,
                               TOF_Frame_t *out)
{
    /* 防止空指针访问。 */
    if ((buf == 0) || (out == 0))
    {
        return 0U;
    }

    /* YC02 帧必须严格等于协议定义的固定长度。 */
    if (len != TOF_FRAME_LENGTH)
    {
        return 0U;
    }

    /* 检查协议帧头。
     *
     * 本协议帧头固定为：
     * Byte0 = 0x5A
     * Byte1 = 0x8E
     * Byte2 = 0x17
     */
    if ((buf[0] != 0x5AU) || (buf[1] != 0x8EU) || (buf[2] != 0x17U))
    {
        return 0U;
    }

    /* 最后一个字节为校验和。
     * 对前 TOF_FRAME_LENGTH - 1 个字节计算校验，
     * 并与最后一个字节进行比较。
     */
    if (yc02_checksum(buf, TOF_FRAME_LENGTH - 1U) != buf[TOF_FRAME_LENGTH - 1U])
    {
        return 0U;
    }

    /* 以下字段均按照 YC02 协议的字节偏移进行解析。 */

    /* 归一化飞行时间，16 位小端。 */
    out->norm_tof = read_le16(&buf[3]);

    /* 归一化峰值，16 位小端。 */
    out->norm_peak = read_le16(&buf[5]);

    /* 归一化噪声，24 位小端。 */
    out->norm_noise = read_le24(&buf[7]);

    /* 多次采样/累积相关参数，16 位小端。 */
    out->multshot = read_le16(&buf[10]);

    /* 衰减后的峰值，16 位小端。 */
    out->atten_peak = read_le16(&buf[12]);

    /* 衰减后的噪声，24 位小端。 */
    out->atten_noise = read_le24(&buf[14]);

    /* 参考飞行时间，16 位小端。 */
    out->ref_tof = read_le16(&buf[17]);

    /* 温度数据，单位通常为 0.01°C。
     *
     * 例如：
     * temperature_centi = 4011
     * 表示温度 40.11°C。
     */
    out->temperature_centi = read_le32s(&buf[19]);

    /* 最终计算得到的距离值，单位 mm。
     * 协议中按 16 位小端读取，再转为有符号整数。
     */
    out->distance_mm = (int16_t)read_le16(&buf[23]);

    /* 距离置信度，单字节。 */
    out->confidence = buf[25];

    /* 记录本帧在 MCU 中被成功解析的毫秒时间戳。 */
    out->tick_ms = PLATFORM_GetTickMs();

    return 1U;
}


/* ---------------------------- USART1 初始化 ----------------------------- */

/**
 * @brief 配置 USART1 为 TOF 通讯串口。
 *
 * @param Baudrate USART1 通讯波特率，例如 460800。
 */
void USART_Configure(uint32_t Baudrate)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
    USART_InitTypeDef USART_InitStruct;

    /* 配置 USART1 中断优先级。
     *
     * 优先级设为 0，通常表示较高优先级，
     * 使 TOF 数据接收优先于 USART2 调试发送。
     */
    NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 0x00U;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    /* 打开 GPIOA 时钟。
     * USART1 使用 PA12 与 PA3。
     */
    RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_GPIOA, ENABLE);

    /* PA12 配置为 USART1_TX。
     *
     * PA12 选择 AF1 复用功能后，
     * 由 USART1 外设驱动该引脚输出串口数据。
     */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_1);
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PA3 配置为 USART1_RX。
     *
     * 输入上拉模式可以减少 RX 悬空时误触发。
     */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* 打开 USART1 外设时钟。 */
    RCC_APB1PeriphClockCmd(RCC_APB1ENR_USART1, ENABLE);

    /* 先写入 USART 默认配置。 */
    USART_StructInit(&USART_InitStruct);

    /* 配置波特率。 */
    USART_InitStruct.USART_BaudRate = Baudrate;

    /* 每个数据帧为 8 位数据位。 */
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;

    /* 每帧使用 1 个停止位。 */
    USART_InitStruct.USART_StopBits = USART_StopBits_1;

    /* 不使用奇偶校验。 */
    USART_InitStruct.USART_Parity = USART_Parity_No;

    /* USART1 同时打开接收与发送功能。
     *
     * 当前项目主要使用 RX 接收 TOF 帧，
     * TX 预留给后续向 TOF 模块发送控制命令。
     */
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStruct);

    /* 打开奇偶校验错误中断。 */
    USART_ITConfig(USART1, USART_IT_PE, ENABLE);

    /* 打开串口错误中断，例如帧错误、噪声错误、溢出错误等。 */
    USART_ITConfig(USART1, USART_IT_ERR, ENABLE);

    /* 正式使能 USART1。 */
    USART_Cmd(USART1, ENABLE);
}


/* ---------------------------- USART2 初始化 ----------------------------- */

/**
 * @brief 配置 USART2 为调试 / RS485 串口。
 *
 * 说明：
 * 当前主要用途是把 TOF 数据打印到电脑端；
 * 后续也可以扩展成真正的 RS485 命令收发接口。
 */
static void USART2_DebugConfigure(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
    USART_InitTypeDef USART_InitStruct;

    /* USART2 使用 PA1、PB1、PB0，因此需要打开 GPIOA、GPIOB 时钟。 */
    RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_GPIOB, ENABLE);

    /* PA1 配置为 USART2_TX。
     *
     * 数据路径：
     * MCU PA1 -> USART2_TX -> MAX485 DI -> RS485 总线
     */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_2);
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PB1 配置为 USART2_RX。
     *
     * 数据路径：
     * RS485 总线 -> MAX485 RO -> MCU PB1
     *
     * 当前代码暂未实现 USART2 接收协议，
     * 但硬件引脚先预留好。
     */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_2);
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* PB0 配置为 MAX485 收发方向控制脚。
     *
     * MAX485：
     * DE  = 发送使能，高电平有效；
     * /RE = 接收使能，低电平有效。
     *
     * 由于 DE 与 /RE 硬件并联：
     * PB0 = 0：接收状态；
     * PB0 = 1：发送状态。
     */
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin = RS485_DIR_PIN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(RS485_DIR_PORT, &GPIO_InitStruct);

    /* 上电默认进入接收状态，防止一开始就占用 RS485 总线。 */
    GPIO_WriteBit(RS485_DIR_PORT, RS485_DIR_PIN, Bit_RESET);

    /* 打开 USART2 外设时钟。 */
    RCC_APB1PeriphClockCmd(RCC_APB1ENR_USART2, ENABLE);

    /* 初始化 USART2 默认参数。 */
    USART_StructInit(&USART_InitStruct);

    /* 设置 USART2 调试输出波特率。 */
    USART_InitStruct.USART_BaudRate = USART2_DEBUG_BAUDRATE;

    /* 8 数据位。 */
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;

    /* 1 停止位。 */
    USART_InitStruct.USART_StopBits = USART_StopBits_1;

    /* 无奇偶校验。 */
    USART_InitStruct.USART_Parity = USART_Parity_No;

    /* 预留收发能力。
     * 当前主要使用 TX，
     * 后续可加入 RXNE/IDLE 中断实现 RS485 指令接收。
     */
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStruct);

    /* 初始化阶段关闭 TXE 中断。
     * 只有真的有调试数据入队后才打开。
     */
    USART_ITConfig(USART2, USART_IT_TXE, DISABLE);

    /* 初始化阶段关闭 TC 中断。
     * 只有发送队列已经空、等待最后一个字节发送完成时才打开。
     */
    USART_ITConfig(USART2, USART_IT_TC, DISABLE);

    /* 配置 USART2 中断优先级。
     *
     * 优先级低于 USART1，
     * 防止调试打印干扰 TOF 高速收帧。
     */
    NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 0x02U;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    /* 使能 USART2。 */
    USART_Cmd(USART2, ENABLE);
}


/* --------------------------- USART1 空闲收帧 ---------------------------- */

/**
 * @brief 启动 USART1 的“RXNE + IDLE”收帧机制。
 *
 * 工作逻辑：
 * 1. RXNE 中断：每收到一个字节，将字节写入 Buffer[]；
 * 2. IDLE 中断：串口线路空闲，认为一帧已经结束；
 * 3. 主循环 TOF_Service() 看到 CompleteFlag 后，解析本帧；
 * 4. 解析后再次调用本函数，重新打开下一帧接收。
 *
 * @param MaxLength 本次接收允许的最大长度。
 */
void USART_RxIdle_Start(uint16_t MaxLength)
{
    uint16_t i;

    /* 防止调用者传入的最大长度超过实际缓冲区大小。 */
    if (MaxLength > USART_RX_BUFFER_SIZE)
    {
        MaxLength = USART_RX_BUFFER_SIZE;
    }

    /* 清空本次接收缓冲区，避免调试观察时混入旧数据。 */
    for (i = 0U; i < MaxLength; i++)
    {
        USART_RxStruct.Buffer[i] = 0U;
    }

    /* 保存当前接收允许的最大字节数。 */
    USART_RxStruct.Length = MaxLength;

    /* 当前已收字节数清零。 */
    USART_RxStruct.CurrentCount = 0U;

    /* 清除“本帧已接收完成”标志。 */
    USART_RxStruct.CompleteFlag = 0U;

    /* 清除“接收溢出”标志。 */
    USART_RxStruct.OverflowFlag = 0U;

    /* 读一次数据寄存器，用于清除可能遗留的数据状态。 */
    (void)USART_ReceiveData(USART1);

    /* 清除 RXNE 中断挂起状态。 */
    USART_ClearITPendingBit(USART1, USART_IT_RXNE);

    /* 清除 RXNE 标志。 */
    USART_ClearFlag(USART1, USART_FLAG_RXNE);

    /* 打开 RXNE 中断：
     * 每接收一个字节都会进入 USART1_IRQHandler。
     */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    /* 打开 IDLE 中断：
     * 当一帧结束后线路出现空闲，触发帧完成判断。
     */
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
}


/* ----------------------------- TOF 通讯初始化 --------------------------- */

/**
 * @brief 初始化 TOF 通讯模块和 USART2 调试串口。
 *
 * @param Baudrate TOF 使用的 USART1 波特率。
 */
void TOF_CommInit(uint32_t Baudrate)
{
    /* 初始化待消费帧状态。 */
    g_tof_pending_ready = 0U;

    /* 初始化有效帧状态。 */
    g_tof_frame_ok = 0U;

    /* 清空有效帧累计数量。 */
    g_tof_valid_frame_count = 0U;

    /* 清空错误帧累计数量。 */
    g_tof_bad_frame_count = 0U;

    /* 清空因主循环未及时处理而丢弃的帧数量。 */
    g_tof_dropped_frame_count = 0U;

    /* 初始化最近有效帧时间。
     * 系统后续可以用该值判断 TOF 是否超时失联。
     */
    g_tof_last_valid_tick_ms = PLATFORM_GetTickMs();

    /* 配置 TOF 专用 USART1。 */
    USART_Configure(Baudrate);

    /* 配置 MAX485 调试用 USART2。 */
    USART2_DebugConfigure();

    /* 启动 USART1 的 RXNE + IDLE 接收。 */
    USART_RxIdle_Start(TOF_RX_MAX_LENGTH);
}


/* ---------------------------- TOF 主循环服务 ---------------------------- */

/**
 * @brief 必须在 main 的 while(1) 内高频调用。
 *
 * 功能：
 * 1. 检查 USART1 是否已收完一帧；
 * 2. 校验并解析该帧；
 * 3. 更新最近距离、置信度、统计变量；
 * 4. 将有效帧放入待消费邮箱；
 * 5. 立刻重启下一帧 USART1 接收。
 *
 * 注意：
 * 此函数不应进行大量打印、延时或复杂距离判断，
 * 否则会影响 460800bps 下的连续接收能力。
 */
void TOF_Service(void)
{
    TOF_Frame_t frame;
    uint16_t count;

    /* 如果当前还没有接收到完整帧，则直接退出。 */
    if (USART_RxStruct.CompleteFlag == 0U)
    {
        return;
    }

    /* 保存本帧实际接收字节数量。 */
    count = USART_RxStruct.CurrentCount;

    /* 记录最近一次帧长度，用于串口调试和异常排查。 */
    g_tof_last_frame_length = count;

    /* 只有在未溢出，并且帧头、长度、校验和均正确时，
     * 才认定为有效 TOF 帧。
     */
    if ((USART_RxStruct.OverflowFlag == 0U) &&
        (YC02_ParseFrame((const uint8_t *)USART_RxStruct.Buffer, count, &frame) != 0U))
    {
        /* 更新对外提供的“最近一次有效距离”。 */
        g_tof_distance_mm = frame.distance_mm;

        /* 更新对外提供的“最近一次有效置信度”。 */
        g_tof_confidence = frame.confidence;

        /* 标记最近一帧处理成功。 */
        g_tof_frame_ok = 1U;

        /* 有效帧总数加一。 */
        g_tof_valid_frame_count++;

        /* 更新最后有效帧接收时间。 */
        g_tof_last_valid_tick_ms = frame.tick_ms;

        /* 只有待处理邮箱为空时，才写入新帧。 */
        if (g_tof_pending_ready == 0U)
        {
            /* 保存完整解析后的帧。 */
            g_tof_pending_frame = frame;

            /* 通知主业务逻辑“有新帧可取”。 */
            g_tof_pending_ready = 1U;
        }
        else
        {
            /* 上一帧还未被主业务逻辑取走。
             *
             * 此处选择丢弃新帧，保留旧帧。
             * 这样能保证 g_tof_pending_frame 从写入到读取期间不被覆盖。
             */
            g_tof_dropped_frame_count++;
        }
    }
    else
    {
        /* 帧无效：
         * 可能是帧长错误、帧头错位、校验失败或接收溢出。
         */
        g_tof_frame_ok = 0U;

        /* 错误帧统计加一。 */
        g_tof_bad_frame_count++;
    }

    /* 无论本帧有效或无效，都必须尽快重新开启下一帧接收。
     *
     * 这里不能放在复杂业务逻辑之后，
     * 否则连续高速帧到来时容易漏收下一帧。
     */
    USART_RxIdle_Start(TOF_RX_MAX_LENGTH);
}


/* -------------------------- 取走一帧有效 TOF 数据 ----------------------- */

/**
 * @brief 从待处理邮箱中取出一帧完整有效 TOF 数据。
 *
 * @param frame 用户传入的结构体地址，用于接收完整帧。
 *
 * @return
 * 1：成功取到一帧；
 * 0：没有待处理帧，或参数为空。
 */
uint8_t TOF_PopValidFrame(TOF_Frame_t *frame)
{
    /* 指针为空，或当前没有待消费帧，直接返回失败。 */
    if ((frame == 0) || (g_tof_pending_ready == 0U))
    {
        return 0U;
    }

    /* 将待处理帧复制给调用者。 */
    *frame = g_tof_pending_frame;

    /* 标记邮箱为空，允许 TOF_Service() 写入下一帧。 */
    g_tof_pending_ready = 0U;

    return 1U;
}


/* -------------------------- USART2 非阻塞发送 --------------------------- */

/**
 * @brief 向 USART2 调试发送队列压入一个字节。
 *
 * 特点：
 * 1. 不等待硬件发送完成；
 * 2. 不使用阻塞延时；
 * 3. 队列满时丢弃新字节；
 * 4. 绝不因为调试输出影响 TOF 收帧。
 *
 * @param data 待发送的一个字节。
 */
void USART2_DebugWriteByte(uint8_t data)
{
    uint8_t head;
    uint8_t next;

    /* 读取当前写指针。 */
    head = g_debug_tx_head;

    /* 计算写入当前字节后的下一个位置。
     * 通过 & DEBUG_TX_MASK 实现环形回绕。
     */
    next = (uint8_t)((head + 1U) & DEBUG_TX_MASK);

    /* 环形队列判满条件：
     * 下一次写入位置与读指针重合，
     * 说明剩余空间不足。
     */
    if (next == g_debug_tx_tail)
    {
        /* 队列满时不阻塞，仅统计丢弃字节。 */
        g_usart2_debug_drop_count++;
        return;
    }

    /* 将待发送字节写入队列当前写位置。 */
    g_debug_tx_buf[head] = data;

    /* 推进写指针，表示该字节已经入队。 */
    g_debug_tx_head = next;

    /* RS485 发送前必须切换 MAX485 到发送状态。 */
    GPIO_WriteBit(RS485_DIR_PORT, RS485_DIR_PIN, Bit_SET);

    /* 当前重新有数据入队，不再等待上一轮发送结束。 */
    g_debug_tx_tc_wait = 0U;

    /* 关闭 TC 中断。
     * 发送尚未完成，不应该在这里等待 TC。
     */
    USART_ITConfig(USART2, USART_IT_TC, DISABLE);

    /* 打开 TXE 中断。
     * TXE 表示发送数据寄存器为空，
     * USART2_IRQHandler 会逐字节将队列数据写入 USART 寄存器。
     */
    USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
}


/**
 * @brief 向 USART2 非阻塞发送一个以 '\0' 结尾的字符串。
 *
 * @param str C 风格字符串指针。
 */
void USART2_DebugWriteString(const char *str)
{
    /* 防止空指针。 */
    if (str == 0)
    {
        return;
    }

    /* 逐字符入队，直到遇到字符串结束符 '\0'。 */
    while (*str != '\0')
    {
        USART2_DebugWriteByte((uint8_t)*str);
        str++;
    }
}


/**
 * @brief 向 USART2 输出一个 32 位无符号十进制数。
 *
 * 示例：
 * 输入：12345
 * 输出字符："12345"
 *
 * @param value 待输出的无符号整数。
 */
void USART2_DebugWriteU32(uint32_t value)
{
    /* 十进制 uint32_t 最大值为 4294967295，共 10 位。 */
    char buf[10];

    /* 当前暂存数字的位数。 */
    uint8_t count;

    count = 0U;

    /* 特殊处理 0。
     * 因为下面的 while(value != 0U) 对 0 不会进入循环。
     */
    if (value == 0U)
    {
        USART2_DebugWriteByte((uint8_t)'0');
        return;
    }

    /* 逐次取最低十进制位。
     *
     * 例如 value = 123：
     * 第一次：取 3，buf[0] = '3'
     * 第二次：取 2，buf[1] = '2'
     * 第三次：取 1，buf[2] = '1'
     *
     * 此时缓冲区字符顺序是反向的。
     */
    while ((value != 0U) && (count < sizeof(buf)))
    {
        buf[count++] = (char)('0' + (value % 10U));
        value /= 10U;
    }

    /* 逆序发出字符，恢复正常十进制显示顺序。 */
    while (count != 0U)
    {
        count--;
        USART2_DebugWriteByte((uint8_t)buf[count]);
    }
}


/**
 * @brief 输出一个 16 位无符号十进制数。
 *
 * 直接转换为 uint32_t 后复用 USART2_DebugWriteU32()。
 *
 * @param value 待输出的 uint16_t。
 */
void USART2_DebugWriteU16(uint16_t value)
{
    USART2_DebugWriteU32((uint32_t)value);
}


/**
 * @brief 输出一个 16 位有符号十进制数。
 *
 * @param value 待输出的 int16_t。
 */
void USART2_DebugWriteS16(int16_t value)
{
    /* 负数先输出负号。 */
    if (value < 0)
    {
        USART2_DebugWriteByte((uint8_t)'-');

        /* 先提升到 int32_t 再取反，
         * 防止 value = -32768 时，直接 -value 发生溢出。
         */
        USART2_DebugWriteU32((uint32_t)(-(int32_t)value));
    }
    else
    {
        /* 非负数直接按无符号数输出。 */
        USART2_DebugWriteU32((uint32_t)value);
    }
}


/**
 * @brief 输出温度，输入单位为 0.01°C。
 *
 * 示例：
 * temperature_centi = 4011  -> "40.11 C"
 * temperature_centi = -525  -> "-5.25 C"
 *
 * @param temperature_centi 温度值，单位 0.01°C。
 */
void USART2_DebugWriteTemperature(int32_t temperature_centi)
{
    uint32_t abs_value;

    /* 如果温度为负数，先发送负号。 */
    if (temperature_centi < 0)
    {
        USART2_DebugWriteByte((uint8_t)'-');

        /* 使用 -(x + 1) + 1 的方式计算绝对值，
         * 避免最小负数 -2147483648 直接取负溢出。
         */
        abs_value = (uint32_t)(-(temperature_centi + 1));
        abs_value += 1U;
    }
    else
    {
        /* 正数直接转换成无符号数。 */
        abs_value = (uint32_t)temperature_centi;
    }

    /* 输出整数部分。
     *
     * 例如 4011 / 100 = 40。
     */
    USART2_DebugWriteU32(abs_value / 100U);

    /* 输出小数点。 */
    USART2_DebugWriteByte((uint8_t)'.');

    /* 输出小数点后第一位。
     *
     * 例如 4011：
     * 4011 / 10 = 401
     * 401 % 10 = 0
     */
    USART2_DebugWriteByte((uint8_t)('0' + ((abs_value / 10U) % 10U)));

    /* 输出小数点后第二位。
     *
     * 例如 4011 % 10 = 1。
     */
    USART2_DebugWriteByte((uint8_t)('0' + (abs_value % 10U)));

    /* 输出单位。 */
    USART2_DebugWriteString(" C");
}


/* --------------------------- USART2 发送中断 ---------------------------- */

/**
 * @brief USART2 发送中断处理函数。
 *
 * 本函数需要由 USART2_IRQHandler() 调用。
 *
 * 中断分为两个阶段：
 *
 * 阶段 1：TXE 中断
 *   当发送数据寄存器空时，
 *   从软件环形队列取一个字节写入 USART2。
 *
 * 阶段 2：TC 中断
 *   软件缓冲区已经没有数据，
 *   但最后一个字节可能还在移位寄存器中。
 *   等待 TC 后，确认最后一位停止位也已发出，
 *   再将 MAX485 切回接收状态。
 */
void USART2_DebugTxIRQHandler(void)
{
    uint8_t tail;

    /* 判断 USART2 的 TXE 中断是否触发。
     *
     * TXE = 发送数据寄存器为空，
     * 表示硬件可以接收下一个待发送字节。
     */
    if (USART_GetITStatus(USART2, USART_IT_TXE) != RESET)
    {
        /* 读取当前队列读指针。 */
        tail = g_debug_tx_tail;

        /* tail != head 表示队列内还有待发送数据。 */
        if (tail != g_debug_tx_head)
        {
            /* 将一个字节写入 USART2 数据寄存器。
             * USART 硬件随后会自动发送起始位、数据位和停止位。
             */
            USART_SendData(USART2, g_debug_tx_buf[tail]);

            /* 推进读指针，表示这个字节已经交给硬件发送。 */
            g_debug_tx_tail = (uint8_t)((tail + 1U) & DEBUG_TX_MASK);
        }
        else
        {
            /* 软件发送缓冲区已经空。
             *
             * 但最后一个字节可能刚刚写入数据寄存器，
             * 还没有完全从 TX 引脚发出。
             */

            /* 没有更多字节需要送入发送数据寄存器，
             * 因此关闭 TXE 中断。
             */
            USART_ITConfig(USART2, USART_IT_TXE, DISABLE);

            /* 标记需要等待 TC。 */
            g_debug_tx_tc_wait = 1U;

            /* 打开 TC 中断。
             * TC 表示整个最后一个字符，包括停止位，已经真正发送完成。
             */
            USART_ITConfig(USART2, USART_IT_TC, ENABLE);
        }
    }

    /* 判断 USART2 的 TC 中断是否触发。 */
    if (USART_GetITStatus(USART2, USART_IT_TC) != RESET)
    {
        /* 只有同时满足以下条件才切回接收状态：
         * 1. 当前确实正在等待最后一个字节完成；
         * 2. 软件发送队列仍然为空。
         *
         * 第二条非常重要：
         * 若刚好有新数据入队，则继续保持发送状态，
         * 不可提前切换 MAX485 到接收。
         */
        if ((g_debug_tx_tc_wait != 0U) && (g_debug_tx_tail == g_debug_tx_head))
        {
            /* 发送真正完成，关闭 TC 中断。 */
            USART_ITConfig(USART2, USART_IT_TC, DISABLE);

            /* 清除等待发送完成标志。 */
            g_debug_tx_tc_wait = 0U;

            /* MAX485 切换回接收状态。
             *
             * PB0 = 0：
             * DE = 0，关闭驱动器；
             * /RE = 0，打开接收器。
             */
            GPIO_WriteBit(RS485_DIR_PORT, RS485_DIR_PIN, Bit_RESET);
        }
    }    
}
