#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdint.h>

typedef void (*BSP_UART_RxCallback_t)(const uint8_t *data, uint16_t len);

/* USART1: 蓝牙，循环 DMA 接收 + DMA 发送 */
void BSP_UART_SetBtRxCallback(BSP_UART_RxCallback_t cb);
void BSP_UART_Init(void);
uint8_t BSP_UART_BtSend(const uint8_t *data, uint16_t len);
uint8_t BSP_UART_BtTxBusy(void);

/* USART10: VOFA+ 遥测，IT 发送（无需 DMA） */
uint8_t BSP_UART_TelemetrySend(const uint8_t *data, uint16_t len);
uint8_t BSP_UART_TelemetryTxBusy(void);

#endif
