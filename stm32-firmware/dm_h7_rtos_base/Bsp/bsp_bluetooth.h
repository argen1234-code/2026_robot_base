#ifndef BSP_BLUETOOTH_H
#define BSP_BLUETOOTH_H

#include <stdint.h>

/* 按键位 */
#define BT_KEY_FORWARD          ((uint8_t)(1U << 0))
#define BT_KEY_BACKWARD         ((uint8_t)(1U << 1))
#define BT_KEY_LEFT             ((uint8_t)(1U << 2))
#define BT_KEY_RIGHT            ((uint8_t)(1U << 3))
#define BT_KEY_ROTATE_LEFT      ((uint8_t)(1U << 4))
#define BT_KEY_ROTATE_RIGHT     ((uint8_t)(1U << 5))

#define BT_ONLINE_TIMEOUT_MS    3000U

/*
 * 旧手机 App 是“按一次锁存”，不会周期发心跳。
 * 默认 0：有锁存键时保持在线（旧行为，@f 后车会一直走）。
 * 置 1：必须周期发任意帧保持在线，超时清键停车。
 */
#define BT_REQUIRE_HEARTBEAT    0

typedef enum
{
    BT_MODE_REQ_NONE = 0,
    BT_MODE_REQ_INDOOR,
    BT_MODE_REQ_IDLE
} BT_ModeReq_t;

typedef struct
{
    char payload[64];
    uint32_t tick;
    uint8_t pending;
} BT_CfgRequest_t;

void BT_Init(void);

/* 在 USART1 RX 中断上下文调用 */
void BT_ProcessRxData(const uint8_t *data, uint16_t len);

/* 5 ms 任务调用 */
void BT_Update(void);
void BT_ServiceTx(void);

uint8_t BT_IsOnline(void);
uint8_t BT_IsActive(void);
uint8_t BT_GetKeyState(void);
void BT_ClearKeys(void);

uint8_t BT_GetAndClearModeReq(void);
uint8_t BT_IsPowerless(void);
void BT_ClearPowerless(void);
uint8_t BT_GetAndClearResumeRequest(void);

uint8_t BT_GetCfgRequest(BT_CfgRequest_t *out);
void BT_QueueText(const char *text);

uint32_t BT_GetLastRxTick(void);

#endif
