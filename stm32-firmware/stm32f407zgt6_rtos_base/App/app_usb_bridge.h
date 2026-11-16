#ifndef APP_USB_BRIDGE_H
#define APP_USB_BRIDGE_H

#include <stdint.h>

/* Jetson → MCU 的 12 字节帧在 500 ms 内没更新就视为失联（fail-closed） */
#define USB_CMD_TIMEOUT_MS      500U

/* 文本命令（@cfg...）行长上限 */
#define USB_TEXT_LINE_MAX       64U

void USB_Bridge_Init(void);

/* USB CDC 接收回调（中断上下文）：解析 AA55 速度帧 / @ 文本命令 */
void USB_Bridge_OnRx(const uint8_t *data, uint32_t len);

/* 下行速度帧状态 */
uint8_t  USB_Bridge_IsOnline(void);
uint8_t  USB_Bridge_GetMode(void);       /* 2=REMOTE, 3=INDOOR */
float    USB_Bridge_GetVx(void);         /* 固件原始值：负=前进 */
float    USB_Bridge_GetWz(void);
uint32_t USB_Bridge_GetSequence(void);

/* 文本命令：中断里只入队，由 chassis 任务取走执行（配置命令会阻塞） */
uint8_t USB_Bridge_GetTextRequest(char *out, uint16_t max_len);

/* 文本回包 / 遥测发送（CDC 忙时返回 0 并计数丢帧） */
void    USB_Bridge_QueueText(const char *text);
uint8_t USB_Bridge_SendFrame(const uint8_t *buf, uint16_t len);
uint16_t USB_Bridge_GetTxBusyCount(void);

#endif
