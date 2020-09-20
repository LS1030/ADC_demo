#ifndef __bsp_led_H
#define __bsp_led_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "gpio.h"

#define LED0_ON  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, RESET)
#define LED0_OFF HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, SET)
#define LED1_ON  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, RESET)
#define LED1_OFF HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, SET)

#define led_delay_time 1000

extern uint32_t led_time;

#ifdef __cplusplus
}
#endif /*__bsp_led_H */

#endif