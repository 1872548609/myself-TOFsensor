/**
 * @file    main.h
 * @author  MegawinTech Application Team
 * @version V1.0.1
 * @date    17-Nov-2023
 * @brief   This file contains all the system functions
 */

/* Define to prevent recursive inclusion */
#ifndef _MAIN_H_
#define _MAIN_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Files include */
#include "mg32_conf.h"

/* LED */
#define LED_GREEN_PORT      GPIOA
#define LED_GREEN_PIN       GPIO_Pin_10

#define LED_RED_PORT        GPIOA
#define LED_RED_PIN         GPIO_Pin_9

/* KEY */
#define KEY_BLUE_PORT       GPIOA
#define KEY_BLUE_PIN        GPIO_Pin_7

#define KEY_RED_PORT        GPIOA
#define KEY_RED_PIN         GPIO_Pin_8

/* OUTPUT */
#define OUTPUT_PORT         GPIOA
#define OUTPUT_PIN          GPIO_Pin_11


/* Exported types *****************************************************************************************************/

/* Exported constants *************************************************************************************************/

/* Exported macro *****************************************************************************************************/

/* Exported variables *************************************************************************************************/

/* Exported functions *************************************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* _MAIN_H_ */

