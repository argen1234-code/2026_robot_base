#ifndef BSP_TIME_H
#define BSP_TIME_H
#include <stdint.h>
#include <stdbool.h>
/* TIM2 free-running 1 MHz counter. No IRQ, pin, RTOS or DMA. */
bool BspTime_Init(void);
uint32_t BspTime_Us(void);
bool BspTime_DebugFrozen(void);
#endif
