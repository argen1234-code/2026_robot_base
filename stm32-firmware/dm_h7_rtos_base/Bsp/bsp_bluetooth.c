#include "bsp_bluetooth.h"
#include "bsp_uart.h"
#include "main.h"
#include <string.h>

#define BT_FRAME_MAX_LEN    64U
#define BT_TX_QUEUE_DEPTH   4U
#define BT_TX_LINE_LEN      96U

static char s_frame[BT_FRAME_MAX_LEN];
static uint16_t s_frame_len = 0U;
static uint8_t s_frame_active = 0U;
static uint8_t s_frame_saw_cr = 0U;

static volatile uint8_t s_key_state = 0U;
static volatile uint8_t s_mode_req = BT_MODE_REQ_NONE;
static volatile uint8_t s_powerless_latched = 0U;
static volatile uint8_t s_resume_req = 0U;
static volatile uint32_t s_last_rx_tick = 0U;

static volatile uint8_t s_ack_pending = 0U;
static volatile uint8_t s_err_pending = 0U;

/* 写入/读取都在关中断临界区内完成，不需要 volatile */
static BT_CfgRequest_t s_cfg_req;

static char s_tx_queue[BT_TX_QUEUE_DEPTH][BT_TX_LINE_LEN];
static uint8_t s_tx_head = 0U;
static uint8_t s_tx_tail = 0U;
static uint8_t s_tx_count = 0U;

/* ------------------------------------------------------------------------- */
/* 内部工具                                                                   */
/* ------------------------------------------------------------------------- */

static uint8_t bt_is_token_char(char c)
{
    return (uint8_t)(((c >= 'a') && (c <= 'z')) ||
                     ((c >= '0') && (c <= '9')) ||
                     (c == '_'));
}

static void bt_lowercase(char *p)
{
    while (*p != '\0')
    {
        if ((*p >= 'A') && (*p <= 'Z'))
        {
            *p = (char)(*p - 'A' + 'a');
        }
        p++;
    }
}

static void bt_copy_string(char *dst, const char *src, uint16_t size)
{
    uint16_t i = 0U;

    if ((dst == NULL) || (src == NULL) || (size == 0U))
    {
        return;
    }

    while ((src[i] != '\0') && (i < (uint16_t)(size - 1U)))
    {
        dst[i] = src[i];
        i++;
    }
    dst[i] = '\0';
}

static void bt_queue_cfg_request(const char *payload)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();

    if (s_cfg_req.pending == 0U)
    {
        bt_copy_string(s_cfg_req.payload, payload, sizeof(s_cfg_req.payload));
        s_cfg_req.tick = HAL_GetTick();
        s_cfg_req.pending = 1U;
    }
    else
    {
        s_err_pending = 1U;
    }

    __set_PRIMASK(primask);
}

static void bt_key_toggle(uint8_t bit)
{
    s_key_state = (uint8_t)(s_key_state ^ bit);
    s_mode_req = BT_MODE_REQ_INDOOR;
}

static uint8_t bt_process_token(const char *token)
{
    if ((strcmp(token, "f") == 0) || (strcmp(token, "forward") == 0))
    {
        bt_key_toggle(BT_KEY_FORWARD);
        return 1U;
    }
    if ((strcmp(token, "b") == 0) || (strcmp(token, "back") == 0) ||
        (strcmp(token, "backward") == 0))
    {
        bt_key_toggle(BT_KEY_BACKWARD);
        return 1U;
    }
    if ((strcmp(token, "l") == 0) || (strcmp(token, "left") == 0))
    {
        bt_key_toggle(BT_KEY_LEFT);
        return 1U;
    }
    if ((strcmp(token, "r") == 0) || (strcmp(token, "right") == 0))
    {
        bt_key_toggle(BT_KEY_RIGHT);
        return 1U;
    }
    if ((strcmp(token, "q") == 0) || (strcmp(token, "ccw") == 0) ||
        (strcmp(token, "rotate_left") == 0))
    {
        bt_key_toggle(BT_KEY_ROTATE_LEFT);
        return 1U;
    }
    if ((strcmp(token, "e") == 0) || (strcmp(token, "cw") == 0) ||
        (strcmp(token, "rotate_right") == 0))
    {
        bt_key_toggle(BT_KEY_ROTATE_RIGHT);
        return 1U;
    }
    if ((strcmp(token, "s") == 0) || (strcmp(token, "stop") == 0))
    {
        BT_ClearKeys();
        s_mode_req = BT_MODE_REQ_INDOOR;
        return 1U;
    }
    if ((strcmp(token, "i") == 0) || (strcmp(token, "indoor") == 0) ||
        (strcmp(token, "bt") == 0) || (strcmp(token, "bluetooth") == 0))
    {
        s_mode_req = BT_MODE_REQ_INDOOR;
        return 1U;
    }
    if (strcmp(token, "idle") == 0)
    {
        BT_ClearKeys();
        s_mode_req = BT_MODE_REQ_IDLE;
        return 1U;
    }

    /* 旧 App 的 GPS/道路/航点命令：本工程未实现，识别但不处理 */
    if ((strcmp(token, "g") == 0) || (strcmp(token, "gps") == 0) ||
        (strncmp(token, "road_", 5) == 0) ||
        (strcmp(token, "gps_add") == 0) ||
        (strcmp(token, "gps_clear") == 0) ||
        (strcmp(token, "gps_point") == 0) ||
        (strcmp(token, "gps_sample") == 0) ||
        (strcmp(token, "remote_add") == 0))
    {
        return 1U;
    }

    return 0U;
}

static void bt_process_payload(char *payload)
{
    char *p = payload;
    char token[16];
    uint8_t token_len = 0U;
    uint8_t any_ok = 0U;
    uint8_t any_err = 0U;
    uint8_t i;

    bt_lowercase(payload);

    while ((*p == ' ') || (*p == '\t'))
    {
        p++;
    }

    if (*p == '\0')
    {
        return;
    }

    if ((strcmp(p, "!") == 0) || (strcmp(p, "emergency") == 0))
    {
        s_powerless_latched = 1U;
        BT_ClearKeys();
        s_mode_req = BT_MODE_REQ_IDLE;
        s_ack_pending = 1U;
        return;
    }

    if ((strcmp(p, "~") == 0) || (strcmp(p, "resume") == 0) ||
        (strcmp(p, "power_on") == 0))
    {
        s_powerless_latched = 0U;
        s_resume_req = 1U;
        BT_ClearKeys();
        s_mode_req = BT_MODE_REQ_INDOOR;
        s_last_rx_tick = HAL_GetTick();
        s_ack_pending = 1U;
        return;
    }

    if (strncmp(p, "cfg", 3) == 0)
    {
        bt_queue_cfg_request(p);
        return;
    }

    /* 按非字母数字下划线切 token，兼容旧 App 的连写命令 */
    for (i = 0U; ; i++)
    {
        char c = p[i];

        if (bt_is_token_char(c))
        {
            if (token_len < (uint8_t)(sizeof(token) - 1U))
            {
                token[token_len++] = c;
            }
        }
        else
        {
            if (token_len > 0U)
            {
                token[token_len] = '\0';
                if (bt_process_token(token) != 0U)
                {
                    any_ok = 1U;
                }
                else
                {
                    any_err = 1U;
                }
                token_len = 0U;
            }

            if (c == '\0')
            {
                break;
            }
        }
    }

    if (any_err != 0U)
    {
        s_err_pending = 1U;
    }
    else if (any_ok != 0U)
    {
        s_ack_pending = 1U;
    }
}

/* ------------------------------------------------------------------------- */
/* 帧解析（USART1 RX 中断上下文）                                             */
/* ------------------------------------------------------------------------- */

void BT_ProcessRxData(const uint8_t *data, uint16_t len)
{
    uint16_t i;

    if (data == NULL)
    {
        return;
    }

    for (i = 0U; i < len; i++)
    {
        char ch = (char)data[i];

        if (s_frame_active == 0U)
        {
            if (ch == '@')
            {
                s_frame_active = 1U;
                s_frame_len = 0U;
                s_frame_saw_cr = 0U;
            }
            continue;
        }

        if (ch == '@')
        {
            /* 新帧头，丢弃半帧 */
            s_frame_len = 0U;
            s_frame_saw_cr = 0U;
            continue;
        }

        if (s_frame_saw_cr != 0U)
        {
            if (ch == '\n')
            {
                s_frame[s_frame_len] = '\0';
                s_last_rx_tick = HAL_GetTick();
                bt_process_payload(s_frame);
                s_frame_active = 0U;
                s_frame_len = 0U;
                s_frame_saw_cr = 0U;
            }
            else
            {
                /* \r 后面不是 \n，整帧作废 */
                s_frame_active = 0U;
                s_frame_len = 0U;
                s_frame_saw_cr = 0U;
                s_err_pending = 1U;
            }
            continue;
        }

        if (ch == '\r')
        {
            s_frame_saw_cr = 1U;
            continue;
        }

        if (s_frame_len < (BT_FRAME_MAX_LEN - 1U))
        {
            s_frame[s_frame_len++] = ch;
        }
        else
        {
            /* 超长帧丢弃 */
            s_frame_active = 0U;
            s_frame_len = 0U;
            s_frame_saw_cr = 0U;
            s_err_pending = 1U;
        }
    }
}

/* ------------------------------------------------------------------------- */
/* 任务侧接口                                                                 */
/* ------------------------------------------------------------------------- */

void BT_Init(void)
{
    s_frame_len = 0U;
    s_frame_active = 0U;
    s_frame_saw_cr = 0U;
    s_key_state = 0U;
    s_mode_req = BT_MODE_REQ_NONE;
    s_powerless_latched = 0U;
    s_resume_req = 0U;
    s_last_rx_tick = 0U;
    s_ack_pending = 0U;
    s_err_pending = 0U;
    s_cfg_req.pending = 0U;
    s_tx_head = 0U;
    s_tx_tail = 0U;
    s_tx_count = 0U;

    BSP_UART_SetBtRxCallback(BT_ProcessRxData);
    BSP_UART_Init();
}

void BT_Update(void)
{
    uint32_t now = HAL_GetTick();

    if (s_last_rx_tick == 0U)
    {
        return;
    }

    if ((now - s_last_rx_tick) > BT_ONLINE_TIMEOUT_MS)
    {
#if BT_REQUIRE_HEARTBEAT
        BT_ClearKeys();
#endif
    }
}

uint8_t BT_IsOnline(void)
{
    uint32_t now;

    if (s_last_rx_tick == 0U)
    {
        return 0U;
    }

    now = HAL_GetTick();
    if ((now - s_last_rx_tick) <= BT_ONLINE_TIMEOUT_MS)
    {
        return 1U;
    }

#if BT_REQUIRE_HEARTBEAT
    return 0U;
#else
    /* 旧锁存协议：有键就认为在线 */
    return (s_key_state != 0U) ? 1U : 0U;
#endif
}

uint8_t BT_IsActive(void)
{
    return (s_key_state != 0U) ? 1U : 0U;
}

uint8_t BT_GetKeyState(void)
{
    return s_key_state;
}

void BT_ClearKeys(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    s_key_state = 0U;
    __set_PRIMASK(primask);
}

uint8_t BT_GetAndClearModeReq(void)
{
    uint8_t req = s_mode_req;

    s_mode_req = BT_MODE_REQ_NONE;
    return req;
}

uint8_t BT_IsPowerless(void)
{
    return s_powerless_latched;
}

void BT_ClearPowerless(void)
{
    s_powerless_latched = 0U;
}

uint8_t BT_GetAndClearResumeRequest(void)
{
    uint8_t ret = s_resume_req;

    s_resume_req = 0U;
    return ret;
}

uint8_t BT_GetCfgRequest(BT_CfgRequest_t *out)
{
    uint8_t ret = 0U;
    uint32_t primask;

    if (out == NULL)
    {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    if (s_cfg_req.pending != 0U)
    {
        *out = s_cfg_req;
        s_cfg_req.pending = 0U;
        ret = 1U;
    }

    __set_PRIMASK(primask);
    return ret;
}

void BT_QueueText(const char *text)
{
    uint32_t primask;

    if (text == NULL)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    if (s_tx_count < BT_TX_QUEUE_DEPTH)
    {
        bt_copy_string(s_tx_queue[s_tx_tail], text, BT_TX_LINE_LEN);
        s_tx_tail = (uint8_t)((s_tx_tail + 1U) % BT_TX_QUEUE_DEPTH);
        s_tx_count++;
    }

    __set_PRIMASK(primask);
}

void BT_ServiceTx(void)
{
    if (BSP_UART_BtTxBusy() != 0U)
    {
        return;
    }

    if (s_err_pending != 0U)
    {
        s_err_pending = 0U;
        BT_QueueText("error\r\n");
    }
    else if (s_ack_pending != 0U)
    {
        s_ack_pending = 0U;
        BT_QueueText("ok\r\n");
    }

    if (s_tx_count == 0U)
    {
        return;
    }

    if (BSP_UART_BtSend((const uint8_t *)s_tx_queue[s_tx_head],
                        (uint16_t)strlen(s_tx_queue[s_tx_head])) != 0U)
    {
        s_tx_head = (uint8_t)((s_tx_head + 1U) % BT_TX_QUEUE_DEPTH);
        s_tx_count--;
    }
}

uint32_t BT_GetLastRxTick(void)
{
    return s_last_rx_tick;
}
