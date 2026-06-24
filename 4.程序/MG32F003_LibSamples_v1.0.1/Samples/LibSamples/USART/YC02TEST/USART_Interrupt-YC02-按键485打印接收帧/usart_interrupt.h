#ifndef _USART_INTERRUPT_H_
#define _USART_INTERRUPT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mg32_conf.h"

#define USART_RX_BUFFER_SIZE      255U
#define TOF_FRAME_LENGTH          27U
#define TOF_RX_MAX_LENGTH         30U

/* USART2 仅用于当前调试：经板载 MAX485 回传至电脑。 */
#define USART2_DEBUG_BAUDRATE     115200U

typedef struct
{
    uint8_t  Buffer[USART_RX_BUFFER_SIZE];
    uint16_t Length;
    uint16_t CurrentCount;
    uint8_t  CompleteFlag;
    uint8_t  OverflowFlag;
} USART_RxTx_TypeDef;

#undef EXTERN
#ifdef _USART_INTERRUPT_C_
#define EXTERN
#else
#define EXTERN extern
#endif

EXTERN volatile USART_RxTx_TypeDef USART_RxStruct;
EXTERN volatile USART_RxTx_TypeDef USART_TxStruct;

EXTERN volatile int16_t  g_tof_distance_mm;
EXTERN volatile uint8_t  g_tof_confidence;
EXTERN volatile uint8_t  g_tof_frame_ok;
EXTERN volatile uint16_t g_tof_last_frame_length;
EXTERN volatile uint32_t g_tof_valid_frame_count;
EXTERN volatile uint32_t g_tof_bad_frame_count;

/* 最近完成的一帧原始数据快照；蓝键按下时由 USART2 回传。 */
EXTERN volatile uint8_t  g_tof_last_frame[TOF_RX_MAX_LENGTH];
EXTERN volatile uint16_t g_tof_last_frame_count;
EXTERN volatile uint8_t  g_tof_last_frame_overflow;

void USART_Configure(uint32_t Baudrate);
void USART_RxIdle_Start(uint16_t MaxLength);
void USART_Interrupt_Sample(void);

#ifdef __cplusplus
}
#endif

#endif /* _USART_INTERRUPT_H_ */
