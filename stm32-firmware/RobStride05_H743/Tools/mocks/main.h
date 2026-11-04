#ifndef TEST_MAIN_H
#define TEST_MAIN_H
#include <stdint.h>
typedef struct {uint32_t CCR;} MockSCB;
extern MockSCB mock_scb;
#define SCB (&mock_scb)
#define SCB_CCR_DC_Msk (1U<<16)
#define __DMB() ((void)0)
uint32_t HAL_GetTick(void);
#endif
