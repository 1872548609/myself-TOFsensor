#ifndef _USART_INTERRUPT_H_
#define _USART_INTERRUPT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mg32_conf.h"

#define USART_RX_BUFFER_SIZE      255U
#define TOF_FRAME_LENGTH          27U
#define TOF_RX_MAX_LENGTH         30U

typedef struct
{
    uint8_t  Buffer[USART_RX_BUFFER_SIZE];
    uint16_t Length;          /* 本帧最大允许接收长度 */
    uint16_t CurrentCount;    /* 本帧实际接收字节数 */
    uint8_t  CompleteFlag;    /* 1：接收到 IDLE，帧结束 */
    uint8_t  OverflowFlag;    /* 1：超过 Length */
} USART_RxTx_TypeDef;

#undef EXTERN
#ifdef _USART_INTERRUPT_C_
#define EXTERN
#else
#define EXTERN extern
#endif

EXTERN volatile USART_RxTx_TypeDef USART_RxStruct;
EXTERN volatile USART_RxTx_TypeDef USART_TxStruct;

/*
 * 不再经 USART1 printf 输出调试文本；使用 Keil Watch 观察这些变量。
 * g_tof_frame_ok: 1 表示最近完整帧校验并解析成功。
 */
EXTERN volatile int16_t  g_tof_distance_mm;
EXTERN volatile uint8_t  g_tof_confidence;
EXTERN volatile uint8_t  g_tof_frame_ok;
EXTERN volatile uint16_t g_tof_last_frame_length;
EXTERN volatile uint32_t g_tof_valid_frame_count;
EXTERN volatile uint32_t g_tof_bad_frame_count;

void USART_Configure(uint32_t Baudrate);
void USART_RxIdle_Start(uint16_t MaxLength);
void USART_Interrupt_Sample(void);

#ifdef __cplusplus
}
#endif

#endif /* _USART_INTERRUPT_H_ */
