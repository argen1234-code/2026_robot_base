#ifndef BSP_WS2812_H
#define BSP_WS2812_H
#include <stdbool.h>
#include <stdint.h>

typedef struct { uint8_t r,g,b; } LedPixel;
enum { LED_TX_IDLE=0, LED_TX_BUSY=1, LED_TX_ERROR=2 };
enum { LED_ERR_NONE=0, LED_ERR_CLOCK=1, LED_ERR_CACHE=2,
       LED_ERR_MEMORY=3, LED_ERR_STREAM_BUSY=4,
       LED_ERR_DMA=5, LED_ERR_TIMEOUT=6 };

/* Owns PA6 AF2, TIM3 CH1+UP, DMA1 Stream0, DMAMUX1 channel0.
 * No IRQ, delay, cache maintenance, shared DMA reset or wait loop. */
bool BspWs2812_Init(void);
uint32_t BspWs2812_Poll(uint32_t now_ms);
bool BspWs2812_Send(const LedPixel *pixels,uint8_t brightness,
                    uint32_t rgb_order,uint32_t now_ms);
uint32_t BspWs2812_LastError(void);
#endif
