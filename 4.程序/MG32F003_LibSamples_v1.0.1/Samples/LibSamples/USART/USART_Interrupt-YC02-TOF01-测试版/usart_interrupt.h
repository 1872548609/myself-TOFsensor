#ifndef _USART_INTERRUPT_H_
#define _USART_INTERRUPT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mg32_conf.h"

/* YC02 固定帧：5A 8E 17 + 23 字节有效字段 + 1 校验。 */
#define TOF_FRAME_LENGTH                 27U
#define TOF_RX_MAX_LENGTH                30U
#define USART_RX_BUFFER_SIZE             64U
#define TOF_DEFAULT_BAUDRATE             460800U

/* USART2 走 MAX485，仅做工厂测试/响应时间日志。 */
#define USART2_DEBUG_BAUDRATE            115200U
#define USART2_DEBUG_TX_BUFFER_SIZE      128U  /* 必须为 2 的幂。 */

typedef struct
{
    uint8_t Buffer[USART_RX_BUFFER_SIZE];
    uint16_t Length;
    uint16_t CurrentCount;
    uint8_t CompleteFlag;
    uint8_t OverflowFlag;
} USART_RxTx_TypeDef;

typedef struct
{
    uint16_t norm_tof;
    uint16_t norm_peak;
    uint32_t norm_noise;
    uint16_t multshot;
    uint16_t atten_peak;
    uint32_t atten_noise;
    uint16_t ref_tof;
    int32_t temperature_centi;
    int16_t distance_mm;   /* YC02 cal_tof，单位 mm。 */
    uint8_t confidence;
    uint32_t tick_ms;      /* 完整有效帧进入软件的时间戳。 */
} TOF_Frame_t;

#undef EXTERN
#ifdef _USART_INTERRUPT_C_
#define EXTERN
#else
#define EXTERN extern
#endif

EXTERN volatile USART_RxTx_TypeDef USART_RxStruct;
EXTERN volatile int16_t g_tof_distance_mm;
EXTERN volatile uint8_t g_tof_confidence;
EXTERN volatile uint8_t g_tof_frame_ok;
EXTERN volatile uint16_t g_tof_last_frame_length;
EXTERN volatile uint32_t g_tof_valid_frame_count;
EXTERN volatile uint32_t g_tof_bad_frame_count;
EXTERN volatile uint32_t g_tof_dropped_frame_count;
EXTERN volatile uint32_t g_tof_last_valid_tick_ms;
EXTERN volatile uint32_t g_usart2_debug_drop_count;

void USART_Configure(uint32_t Baudrate);
void USART_RxIdle_Start(uint16_t MaxLength);

/* 产品主循环接口。 */
void TOF_CommInit(uint32_t Baudrate);
void TOF_Service(void);
uint8_t TOF_PopValidFrame(TOF_Frame_t *frame);

/* USART2 工厂调试接口，全部为非阻塞入队。 */
void USART2_DebugWriteByte(uint8_t data);
void USART2_DebugWriteString(const char *str);
void USART2_DebugWriteU32(uint32_t value);
void USART2_DebugWriteU16(uint16_t value);
void USART2_DebugWriteS16(int16_t value);
void USART2_DebugWriteTemperature(int32_t temperature_centi);
void USART2_DebugTxIRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* _USART_INTERRUPT_H_ */
