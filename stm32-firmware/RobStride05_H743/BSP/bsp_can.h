#ifndef BSP_CAN_H
#define BSP_CAN_H
#include <stdint.h>
#include <stdbool.h>
typedef struct { uint32_t id,tick_ms; uint8_t data[8]; } CanFrame;
typedef struct {
 uint32_t rx,tx_ok,tx_fail,rx_drop,errors,bus_off,recoveries;
 uint32_t last_hal_error,tec,rec;
 uint32_t rx_irq,rx_other;     /* FIFO callbacks / frames outside the private protocol format. */
 uint32_t last_rx_id,last_rx_dlc,last_rx_extended;
 uint8_t last_rx_data[8];      /* Last raw frame, including frames the protocol rejects. */
} CanStats;
enum {CAN_TX_IDLE=0,CAN_TX_BUSY=1,CAN_TX_OK=2,CAN_TX_FAIL=3};
bool BspCan_Start(void);
bool BspCan_SendBegin(const CanFrame *frame);
uint32_t BspCan_SendPoll(void); /* No waits; OK/FAIL consumes completion. */
void BspCan_Cancel(void);
void BspCan_DiscardRx(void);
bool BspCan_Receive(CanFrame *frame);
bool BspCan_IsBusOff(void);
bool BspCan_Recover(void);
void BspCan_GetStats(CanStats *out);
#endif
