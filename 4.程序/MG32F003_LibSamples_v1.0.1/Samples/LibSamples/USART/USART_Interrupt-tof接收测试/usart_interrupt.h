/**
 * @file    usart_interrupt.h
 * @author  MegawinTech Application Team
 * @version V1.0.1
 * @date    14-Nov-2023
 * @brief   This file contains all the system functions
 */

/* Define to prevent recursive inclusion */
#ifndef _USART_INTERRUPT_H_
#define _USART_INTERRUPT_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Files include */
#include "mg32_conf.h"

#define USART_RX_BUFFER_SIZE    255

/* Exported types *****************************************************************************************************/
typedef struct
{
    uint8_t  Buffer[USART_RX_BUFFER_SIZE];
    uint16_t Length;          // 允许接收的最大长度，可自定义
    uint16_t CurrentCount;    // 当前实际收到的字节数
    uint8_t  CompleteFlag;    // 1：一帧接收完成
    uint8_t  OverflowFlag;    // 1：缓冲区溢出
} USART_RxTx_TypeDef;

/* Exported constants *************************************************************************************************/

/* Exported macro *****************************************************************************************************/

/* Exported variables *************************************************************************************************/
#undef EXTERN

#ifdef _USART_INTERRUPT_C_
#define EXTERN
#else
#define EXTERN extern
#endif

EXTERN volatile USART_RxTx_TypeDef USART_RxStruct;
EXTERN volatile USART_RxTx_TypeDef USART_TxStruct;

/* Exported functions *************************************************************************************************/
void USART_Interrupt_Sample(void);

#ifdef __cplusplus
}
#endif

#endif /* _USART_INTERRUPT_H_ */

