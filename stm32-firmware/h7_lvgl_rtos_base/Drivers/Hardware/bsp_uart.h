#ifndef BSP_USART_H
#define BSP_USART_H

#include "main.h"

// 发送缓冲区：用于 printf 类调试输出
#define UART_TX_BUFFER_SIZE 256
// 接收缓冲区：GPS NMEA/二进制报文会明显长于 5 字节，因此扩到 512
#define UART_RX_BUFFER_SIZE 512

#define BLOCK_WAITING_TIME 1000

typedef enum
{
	UART_DMA_RX = 0,
	UART_DMA_ToIdle_RX,
	UART_IT_RX,
	UART_IT_ToIdle_RX,
	UART_Block_RX
} UART_RX_MODE;

typedef enum
{
	UART_DMA_TX = 0,
	UART_IT_TX,
	UART_Block_TX
} UART_TX_MODE;

extern void uart_init(UART_HandleTypeDef *huart, uint8_t uart_mode);
extern int my_uart_printf(UART_HandleTypeDef *huart, uint8_t send_mode, const char *format, ...);

#endif
