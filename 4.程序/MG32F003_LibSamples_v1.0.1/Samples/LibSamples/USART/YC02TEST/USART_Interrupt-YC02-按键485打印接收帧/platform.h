#ifndef _PLATFORM_H_
#define _PLATFORM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mg32_conf.h"

/*
 * 本板实际映射：
 * LED1 -> PA10 绿灯，高电平点亮
 * LED2 -> PA9  红灯，高电平点亮
 * LED3 / LED4 未使用，仅为兼容原示例枚举保留。
 */
typedef enum
{
    LED1,
    LED2,
    LED3,
    LED4
} LEDn_TypeDef;

#undef EXTERN
#ifdef _PLATFORM_C_
#define EXTERN
#else
#define EXTERN extern
#endif

EXTERN volatile uint32_t PLATFORM_DelayTick;

void PLATFORM_DelayMS(uint32_t Millisecond);
void PLATFORM_LED_Enable(LEDn_TypeDef LEDn, FunctionalState State);
void PLATFORM_LED_Toggle(LEDn_TypeDef LEDn);
void PLATFORM_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* _PLATFORM_H_ */
