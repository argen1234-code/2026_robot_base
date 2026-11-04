/* ============================================================================
 * bsp_can.c —— FDCAN1 收发（经典 CAN、29 位扩展帧、8 字节）
 *
 * 设计原则：中断里只做"搬字节"，解析和决策全部留给任务。
 *   接收：RX FIFO0 中断 → 校验帧格式 → 拷进 32 帧的软件环形队列 → 返回
 *         （单次中断最多取 16 帧，即硬件 FIFO 深度，绝不无限循环）
 *   发送：任务调 SendBegin 把帧放进 TX FIFO，之后用 SendPoll 轮询是否真的
 *         上了总线（看 TXBTO）。"进了 FIFO"不等于"发出去了"，所以要轮询。
 *
 * 并发安全：队列的 head/tail 在读写时短暂关总中断（__disable_irq/__set_PRIMASK），
 * 因为 ISR 和任务都会访问。这个临界区只有几条指令，不会影响时序。
 *
 * 为什么不用 RTOS 队列：本文件在 HAL 与 RTOS 两个工程里是同一份源码，
 * 用"关中断 + 环形缓冲"可以完全不依赖 RTOS，也避免 ISR 里调用 FromISR API。
 * ==========================================================================*/
#include "bsp_can.h"
#include "app_config.h"
#include "fdcan.h"
#include <string.h>

#define RX_COUNT 32U
static CanFrame rx[RX_COUNT];
static volatile uint32_t head,tail;
static volatile CanStats stats;
static uint32_t tx_slot,tx_start;
static bool tx_active,started;
/* 配置过滤器/全局过滤/中断线并启动 FDCAN。
 * 过滤器掩码为 0（全收）：因为电机身份应答（类型 0）的目的地址是 0xFE，
 * 不是正常的主机地址 0xFD，早期联调时不能把它过滤掉。ID 的合法性由协议层判断。 */
static bool configure_start(void)
{
 FDCAN_FilterTypeDef f={0};
 f.IdType=FDCAN_EXTENDED_ID;f.FilterIndex=0;f.FilterType=FDCAN_FILTER_MASK;
 f.FilterConfig=FDCAN_FILTER_TO_RXFIFO0;
 /* Official sample receives first and checks motor ID in the protocol layer.
  * A zero mask also admits type-0 identity replies, whose destination is FE,
  * not the normal host FD. Do not hide unexpected IDs during commissioning. */
 f.FilterID1=0;f.FilterID2=0;
 if(HAL_FDCAN_ConfigFilter(&hfdcan1,&f)!=HAL_OK ||
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,FDCAN_ACCEPT_IN_RX_FIFO0,FDCAN_ACCEPT_IN_RX_FIFO0,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE)!=HAL_OK ||
    HAL_FDCAN_ConfigInterruptLines(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE|FDCAN_IT_RX_FIFO0_MESSAGE_LOST|FDCAN_IT_BUS_OFF,FDCAN_INTERRUPT_LINE0)!=HAL_OK ||
    HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE|FDCAN_IT_RX_FIFO0_MESSAGE_LOST|FDCAN_IT_BUS_OFF,0)!=HAL_OK ||
    HAL_FDCAN_Start(&hfdcan1)!=HAL_OK)return false;
 HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
 return true;
}
bool BspCan_Start(void) {started=configure_start();return started;}
bool BspCan_IsBusOff(void)
{
 FDCAN_ProtocolStatusTypeDef p={0};
 return !started || HAL_FDCAN_GetProtocolStatus(&hfdcan1,&p)!=HAL_OK || p.BusOff!=0;
}
bool BspCan_SendBegin(const CanFrame *f)
{
 FDCAN_TxHeaderTypeDef h={0};
 if(tx_active)return false;
 if(BspCan_IsBusOff())goto fail;
 h.Identifier=f->id;h.IdType=FDCAN_EXTENDED_ID;h.TxFrameType=FDCAN_DATA_FRAME;
 h.DataLength=FDCAN_DLC_BYTES_8;h.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
 h.BitRateSwitch=FDCAN_BRS_OFF;h.FDFormat=FDCAN_CLASSIC_CAN;h.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
 if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&h,f->data)!=HAL_OK)goto fail;
 tx_slot=HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan1);tx_start=HAL_GetTick();tx_active=true;
 return true;
fail:
 stats.tx_fail++;stats.last_hal_error=HAL_FDCAN_GetError(&hfdcan1);return false;
}
uint32_t BspCan_SendPoll(void)
{
 if(!tx_active)return CAN_TX_IDLE;
 if(!BspCan_IsBusOff() && !HAL_FDCAN_IsTxBufferMessagePending(&hfdcan1,tx_slot)) {
  tx_active=false;
  if(hfdcan1.Instance->TXBTO & tx_slot){stats.tx_ok++;return CAN_TX_OK;}
 }else if(HAL_GetTick()-tx_start<APP_CAN_TX_TIMEOUT_MS && !BspCan_IsBusOff())return CAN_TX_BUSY;
 (void)HAL_FDCAN_AbortTxRequest(&hfdcan1,tx_slot);tx_active=false;
 stats.tx_fail++;stats.last_hal_error=HAL_FDCAN_GetError(&hfdcan1);return CAN_TX_FAIL;
}
void BspCan_Cancel(void)
{
 /* Abort pending motion before submitting STOP; no retransmission/catch-up. */
 if(started)(void)HAL_FDCAN_AbortTxRequest(&hfdcan1,FDCAN_TX_BUFFER0|FDCAN_TX_BUFFER1|FDCAN_TX_BUFFER2);
 tx_active=false;
}
bool BspCan_Receive(CanFrame *f)
{
 uint32_t key=__get_PRIMASK();bool found=false;
 __disable_irq();
 if(head!=tail){*f=rx[tail%RX_COUNT];tail++;found=true;}
 __set_PRIMASK(key);return found;
}
void BspCan_DiscardRx(void)
{
 uint32_t key=__get_PRIMASK();__disable_irq();tail=head;
 /* CPU halt may leave old frames in the hardware FIFO as well. */
 for(uint32_t i=0;i<16 && HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1,FDCAN_RX_FIFO0);i++) {
  FDCAN_RxHeaderTypeDef h;uint8_t raw[64];
  if(HAL_FDCAN_GetRxMessage(&hfdcan1,FDCAN_RX_FIFO0,&h,raw)!=HAL_OK)break;
 }
 __set_PRIMASK(key);
}
bool BspCan_Recover(void)
{
 /* Only main context, only disabled, at most once per second. HAL Init/Stop
  * have vendor tick timeouts. No automatic PREPARE/ENABLE after recovery. */
 HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);BspCan_Cancel();started=false;
 (void)HAL_FDCAN_Stop(&hfdcan1);(void)HAL_FDCAN_DeInit(&hfdcan1);
 if(HAL_FDCAN_Init(&hfdcan1)!=HAL_OK)return false;
 BspCan_DiscardRx();started=configure_start();
 if(started)stats.recoveries++;
 return started;
}
void BspCan_GetStats(CanStats *out)
{
 FDCAN_ErrorCountersTypeDef e={0};uint32_t key=__get_PRIMASK();
 __disable_irq();*out=stats;__set_PRIMASK(key);
 if(HAL_FDCAN_GetErrorCounters(&hfdcan1,&e)==HAL_OK){out->tec=e.TxErrorCnt;out->rec=e.RxErrorCnt;}
}
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *h,uint32_t events)
{
 FDCAN_RxHeaderTypeDef rh;uint8_t raw[64];CanFrame f;uint32_t i;
 if(h!=&hfdcan1)return;
 stats.rx_irq++;
 if(events&FDCAN_IT_RX_FIFO0_MESSAGE_LOST){stats.rx_drop++;stats.errors++;}
 /* Bounded ISR: hardware FIFO has 16 entries. Parse in the main loop. */
 for(uint32_t n=0;n<16U && HAL_FDCAN_GetRxFifoFillLevel(h,FDCAN_RX_FIFO0);n++) {
  if(HAL_FDCAN_GetRxMessage(h,FDCAN_RX_FIFO0,&rh,raw)!=HAL_OK){stats.errors++;break;}
  stats.last_rx_id=rh.Identifier;stats.last_rx_dlc=rh.DataLength;
  stats.last_rx_extended=rh.IdType==FDCAN_EXTENDED_ID;
  for(i=0;i<8U;i++)stats.last_rx_data[i]=(rh.RxFrameType==FDCAN_DATA_FRAME && i<rh.DataLength)?raw[i]:0U;
  if(rh.IdType!=FDCAN_EXTENDED_ID || rh.RxFrameType!=FDCAN_DATA_FRAME ||
     rh.DataLength!=FDCAN_DLC_BYTES_8 || rh.FDFormat!=FDCAN_CLASSIC_CAN){stats.rx_other++;continue;}
  memcpy(f.data,raw,8);f.id=rh.Identifier;f.tick_ms=HAL_GetTick();stats.rx++;
  if(head-tail>=RX_COUNT)stats.rx_drop++;
  else {rx[head%RX_COUNT]=f;__DMB();head++;}
 }
}
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *h,uint32_t events)
{if(h==&hfdcan1){stats.errors++;if(events&FDCAN_IT_BUS_OFF)stats.bus_off++;}}
