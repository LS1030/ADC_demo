#ifndef __bsp_debug_H
#define __bsp_debug_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "usart.h"
#include <stdarg.h>

void Uart_printf(UART_HandleTypeDef *huart, char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /*__bsp_debug_H */