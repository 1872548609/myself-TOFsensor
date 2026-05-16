#ifndef _PLATFORM_H_
#define _PLATFORM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mg32_conf.h"
#include "main.h"

typedef enum
{
    BOARD_LED_GREEN = 0,
    BOARD_LED_RED
} BOARD_LED_TypeDef;

typedef enum
{
    BOARD_KEY_BLUE = 0,
    BOARD_KEY_RED
} BOARD_KEY_TypeDef;

#undef EXTERN
#ifdef _PLATFORM_C_
#define EXTERN
#else
#define EXTERN extern
#endif

EXTERN volatile uint32_t PLATFORM_DelayTick;

void PLATFORM_Init(void);
void PLATFORM_DelayMS(uint32_t Millisecond);

/* GPIO */
void BOARD_GPIO_Init(void);

/* LED */
void BOARD_LED_On(BOARD_LED_TypeDef led);
void BOARD_LED_Off(BOARD_LED_TypeDef led);
void BOARD_LED_Toggle(BOARD_LED_TypeDef led);

/* KEY */
uint8_t BOARD_KEY_Read(BOARD_KEY_TypeDef key);

/* OUTPUT */
void BOARD_OUTPUT_On(void);
void BOARD_OUTPUT_Off(void);
void BOARD_OUTPUT_Toggle(void);

#ifdef __cplusplus
}
#endif

#endif
