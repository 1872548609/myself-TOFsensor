/**
 * @file    usart_interrupt.h
 * @brief   USART1 接收 TOF 帧，USART2（RS485）输出调试文本
 */
#ifndef _USART_INTERRUPT_H_
#define _USART_INTERRUPT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mg32_conf.h"

/*
 * TOF 原始帧最大长度。
 * 当前 YC02 回传帧为 27 字节，保留到 32 字节便于后续扩展。
 */
#define USART_RX_BUFFER_SIZE        255U
#define TOF_FRAME_MAX_LENGTH        32U

typedef struct
{
    uint8_t  Buffer[USART_RX_BUFFER_SIZE];
    uint16_t Length;          /* 本轮允许接收的最大字节数 */
    uint16_t CurrentCount;    /* 当前实际已收到的字节数 */
    uint8_t  CompleteFlag;    /* 1：收到空闲 IDLE，当前帧冻结 */
    uint8_t  OverflowFlag;    /* 1：接收长度超过 Length */
} USART_RxTx_TypeDef;

#undef EXTERN
#ifdef _USART_INTERRUPT_C_
#define EXTERN
#else
#define EXTERN extern
#endif

/*
 * USART_RxStruct：USART1 的 TOF 接收缓冲区。
 * USART_TxStruct：USART2 的 RS485 文本输出缓冲区。
 */
EXTERN volatile USART_RxTx_TypeDef USART_RxStruct;
EXTERN volatile USART_RxTx_TypeDef USART_TxStruct;

void USART_Interrupt_Sample(void);

#ifdef __cplusplus
}
#endif

#endif /* _USART_INTERRUPT_H_ */
