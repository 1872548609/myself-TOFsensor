#ifndef _PLATFORM_H_
#define _PLATFORM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mg32_conf.h"

typedef enum
{
    LED1 = 0, /* PA10，绿灯，高电平点亮 */
    LED2,     /* PA9，红灯，高电平点亮 */
    LED3,
    LED4
} LEDn_TypeDef;

typedef enum
{
    PLATFORM_KEY1 = 0, /* PA7，蓝键，低电平按下：二段标定 */
    PLATFORM_KEY2      /* PA8，红键，低电平按下：NO/NC 切换 */
} PLATFORM_Key_t;

#undef EXTERN
#ifdef _PLATFORM_C_
#define EXTERN
#else
#define EXTERN extern
#endif

EXTERN volatile uint32_t PLATFORM_DelayTick;
EXTERN volatile uint32_t PLATFORM_SystemTickMs;

void PLATFORM_Init(void);
void PLATFORM_DelayMS(uint32_t Millisecond);
uint32_t PLATFORM_GetTickMs(void);

void PLATFORM_LED_Enable(LEDn_TypeDef LEDn, FunctionalState State);
void PLATFORM_LED_Toggle(LEDn_TypeDef LEDn);

uint8_t PLATFORM_KeyIsPressed(PLATFORM_Key_t key);

/*
 * PA11 -> R6 -> Q1 基极。
 * ENABLE：Q1 导通，NPN 开集电极 OUT 被拉低。
 * DISABLE：Q1 截止，OUT 开路。
 */
void PLATFORM_OutputTransistor(FunctionalState State);

#ifdef __cplusplus
}
#endif

#endif /* _PLATFORM_H_ */
