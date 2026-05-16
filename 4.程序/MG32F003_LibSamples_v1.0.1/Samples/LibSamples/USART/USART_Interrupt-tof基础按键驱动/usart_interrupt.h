#ifndef _USART_INTERRUPT_H_
#define _USART_INTERRUPT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mg32_conf.h"

#define USART_RX_BUFFER_SIZE    255

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

extern volatile USART_RxTx_TypeDef USART1_RxStruct;
extern volatile USART_RxTx_TypeDef USART2_RxStruct;

void USART1_TOF_Init(uint32_t Baudrate);
void USART2_RS485_Init(uint32_t Baudrate);

void USART1_RxIdle_Start(uint16_t MaxLength);
void USART2_RxIdle_Start(uint16_t MaxLength);

void USART1_SendByte(uint8_t Data);
void USART2_SendByte(uint8_t Data);
void USART1_SendBuffer(const uint8_t *buf, uint16_t len);
void USART2_SendBuffer(const uint8_t *buf, uint16_t len);

void USART_Module_Test(void);

#ifdef __cplusplus
}
#endif

#endif

