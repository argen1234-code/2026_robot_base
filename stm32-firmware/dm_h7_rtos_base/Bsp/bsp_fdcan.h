#ifndef BSP_FDCAN_H
#define BSP_FDCAN_H

#include <stdint.h>

/* 一帧经典 CAN（8 字节） */
typedef struct
{
    uint16_t id;
    uint8_t  len;
    uint8_t  data[8];
    uint32_t tick;
} BSP_FDCAN_Frame_t;

/* 接收回调：在 FDCAN1 中断上下文执行，必须短小 */
typedef void (*BSP_FDCAN_RxCallback_t)(const BSP_FDCAN_Frame_t *frame);

typedef struct
{
    uint32_t rx_count;
    uint32_t tx_ok;
    uint32_t tx_fail;
    uint32_t bus_off_count;
    uint32_t error_passive_count;
    uint32_t unexpected_rx;
} BSP_FDCAN_Stats_t;

void BSP_FDCAN_Init(void);
void BSP_FDCAN_SetRxCallback(BSP_FDCAN_RxCallback_t cb);

/* 发送 8 字节标准帧，返回 1=已入队，0=失败（FIFO 满/总线关闭） */
uint8_t BSP_FDCAN_Send(uint16_t std_id, const uint8_t data[8]);

uint8_t BSP_FDCAN_IsBusOff(void);
uint8_t BSP_FDCAN_Recover(void);
void BSP_FDCAN_GetStats(BSP_FDCAN_Stats_t *out);

#endif
