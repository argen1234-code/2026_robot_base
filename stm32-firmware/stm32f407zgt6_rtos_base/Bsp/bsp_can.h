#ifndef BSP_CAN_H
#define BSP_CAN_H

#include <stdint.h>

/* 一帧经典 CAN（8 字节），API 与 dm_h7_rtos_base 的 bsp_fdcan 保持一致 */
typedef struct
{
    uint16_t id;
    uint8_t  len;
    uint8_t  data[8];
    uint32_t tick;
} BSP_CAN_Frame_t;

/* 接收回调：在 CAN1 RX0 中断上下文执行，必须短小 */
typedef void (*BSP_CAN_RxCallback_t)(const BSP_CAN_Frame_t *frame);

typedef struct
{
    uint32_t rx_count;
    uint32_t tx_ok;
    uint32_t tx_fail;
    uint32_t bus_off_count;
    uint32_t error_passive_count;
    uint32_t unexpected_rx;
} BSP_CAN_Stats_t;

void BSP_CAN_Init(void);
void BSP_CAN_SetRxCallback(BSP_CAN_RxCallback_t cb);

/* 发送 8 字节标准帧，返回 1=已入队，0=失败（邮箱满/总线关闭） */
uint8_t BSP_CAN_Send(uint16_t std_id, const uint8_t data[8]);

uint8_t BSP_CAN_IsBusOff(void);
uint8_t BSP_CAN_Recover(void);
void BSP_CAN_GetStats(BSP_CAN_Stats_t *out);

#endif
