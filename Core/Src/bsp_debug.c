#include "bsp_debug.h"

static uint8_t Uart_TxBuff[200];

void Uart_printf(UART_HandleTypeDef *huart, char *fmt, ...)
{
  uint16_t len;
  va_list ap;
  va_start(ap, fmt);
  vsprintf((char*) Uart_TxBuff, fmt, ap);
  va_end(ap);
  len = strlen((const char*) Uart_TxBuff); //此次发送数据的长度
  HAL_UART_Transmit(huart, Uart_TxBuff, len, 100);
}