/*
 * USB CDC 桥接：下行 AA55 速度帧、文本配置命令，上行 CC55 v2 遥测。
 *
 * 下行帧（与 chassis_protocol.py 一致）：
 *     AA 55 | mode(u8) | vx(f32 LE) | wz(f32 LE) | XOR(字节[2..11])
 *     mode: 2=REMOTE, 3=INDOOR；vx 为固件原始约定（负值=前进）
 *
 * 上行遥测由 app_telemetry.c 组帧，这里只负责发送（CDC 忙时丢帧并计数）。
 */
#include "app_usb_bridge.h"
#include "app_motor_config.h"
#include "main.h"
#include "usbd_cdc_if.h"
#include "usbd_def.h"

#include <math.h>
#include <string.h>

/* 帧：2 字节头 + 1 字节 mode + 4 字节 vx + 4 字节 wz + 1 字节校验 = 12 */
#define CMD_BODY_SIZE       10U   /* mode + vx + wz + checksum */

typedef enum
{
    RX_WAIT_HEAD = 0,
    RX_WAIT_MAGIC,
    RX_BODY,
    RX_TEXT
} RxState_t;

static RxState_t s_rx_state = RX_WAIT_HEAD;
static uint8_t   s_rx_buf[CMD_BODY_SIZE];
static uint8_t   s_rx_index = 0U;

static volatile uint8_t  s_cmd_valid = 0U;
static volatile uint8_t  s_cmd_mode = 0U;
static volatile float    s_cmd_vx = 0.0f;
static volatile float    s_cmd_wz = 0.0f;
static volatile uint32_t s_cmd_tick = 0U;
static volatile uint32_t s_cmd_sequence = 0U;

static char     s_text_buf[USB_TEXT_LINE_MAX];
static uint8_t  s_text_len = 0U;
static uint8_t  s_text_overflow = 0U;
static volatile uint8_t s_text_ready = 0U;
static char     s_text_line[USB_TEXT_LINE_MAX];

static volatile uint16_t s_tx_busy_count = 0U;

/* 文本回包缓冲：CDC 发送是异步的，缓冲区必须在发送完成前保持有效 */
static char s_text_tx_buf[192];

void USB_Bridge_Init(void)
{
    s_rx_state = RX_WAIT_HEAD;
    s_rx_index = 0U;
    s_cmd_valid = 0U;
    s_cmd_mode = 0U;
    s_cmd_vx = 0.0f;
    s_cmd_wz = 0.0f;
    s_cmd_tick = 0U;
    s_cmd_sequence = 0U;
    s_text_len = 0U;
    s_text_overflow = 0U;
    s_text_ready = 0U;
    s_tx_busy_count = 0U;
}

static void usb_parse_cmd_frame(const uint8_t body[CMD_BODY_SIZE])
{
    uint8_t checksum = 0U;
    uint8_t mode;
    float vx;
    float wz;
    uint8_t i;

    for (i = 0U; i < (CMD_BODY_SIZE - 1U); i++)
    {
        checksum ^= body[i];
    }

    if (checksum != body[CMD_BODY_SIZE - 1U])
    {
        return;
    }

    mode = body[0];
    if ((mode != 2U) && (mode != 3U))
    {
        return;
    }

    memcpy(&vx, &body[1], sizeof(float));
    memcpy(&wz, &body[5], sizeof(float));

    if (!isfinite(vx) || !isfinite(wz))
    {
        return;
    }

    s_cmd_mode = mode;
    s_cmd_vx = vx;
    s_cmd_wz = wz;
    s_cmd_tick = HAL_GetTick();
    s_cmd_sequence++;
    s_cmd_valid = 1U;
}

static void usb_text_push(uint8_t byte)
{
    if (byte == '\r' || byte == '\n')
    {
        if (s_text_overflow == 0U && s_text_len > 0U)
        {
            s_text_buf[s_text_len] = '\0';
            memcpy(s_text_line, s_text_buf, s_text_len + 1U);
            s_text_ready = 1U;
        }
        s_text_len = 0U;
        s_text_overflow = 0U;
        return;
    }

    if (s_text_len < (USB_TEXT_LINE_MAX - 1U))
    {
        s_text_buf[s_text_len++] = (char)byte;
    }
    else
    {
        s_text_overflow = 1U;
    }
}

void USB_Bridge_OnRx(const uint8_t *data, uint32_t len)
{
    uint32_t i;

    if (data == NULL)
    {
        return;
    }

    for (i = 0U; i < len; i++)
    {
        uint8_t byte = data[i];

        switch (s_rx_state)
        {
        case RX_WAIT_HEAD:
            if (byte == 0xAAU)
            {
                s_rx_state = RX_WAIT_MAGIC;
            }
            else if (byte == '@')
            {
                s_text_len = 0U;
                s_text_overflow = 0U;
                s_rx_state = RX_TEXT;
            }
            break;

        case RX_TEXT:
            if ((byte == '\r') || (byte == '\n'))
            {
                usb_text_push(byte);
                s_rx_state = RX_WAIT_HEAD;
            }
            else
            {
                usb_text_push(byte);
            }
            break;

        case RX_WAIT_MAGIC:
            if (byte == 0x55U)
            {
                s_rx_index = 0U;
                s_rx_state = RX_BODY;
            }
            else if (byte != 0xAAU)
            {
                s_rx_state = RX_WAIT_HEAD;
            }
            break;

        case RX_BODY:
            s_rx_buf[s_rx_index++] = byte;
            if (s_rx_index >= CMD_BODY_SIZE)
            {
                usb_parse_cmd_frame(s_rx_buf);
                s_rx_index = 0U;
                s_rx_state = RX_WAIT_HEAD;
            }
            break;

        default:
            s_rx_state = RX_WAIT_HEAD;
            break;
        }
    }
}

uint8_t USB_Bridge_IsOnline(void)
{
    if (s_cmd_valid == 0U)
    {
        return 0U;
    }

    return ((uint32_t)(HAL_GetTick() - s_cmd_tick) <= USB_CMD_TIMEOUT_MS)
               ? 1U : 0U;
}

uint8_t USB_Bridge_GetMode(void)
{
    return s_cmd_mode;
}

float USB_Bridge_GetVx(void)
{
    return s_cmd_vx;
}

float USB_Bridge_GetWz(void)
{
    return s_cmd_wz;
}

uint32_t USB_Bridge_GetSequence(void)
{
    return s_cmd_sequence;
}

uint8_t USB_Bridge_GetTextRequest(char *out, uint16_t max_len)
{
    uint8_t ready;

    if ((out == NULL) || (max_len == 0U))
    {
        return 0U;
    }

    __disable_irq();
    ready = s_text_ready;
    if (ready != 0U)
    {
        (void)strncpy(out, s_text_line, (size_t)(max_len - 1U));
        out[max_len - 1U] = '\0';
        s_text_ready = 0U;
    }
    __enable_irq();

    return ready;
}

void USB_Bridge_QueueText(const char *text)
{
    size_t len;

    if (text == NULL)
    {
        return;
    }

    len = strlen(text);
    if (len >= sizeof(s_text_tx_buf))
    {
        len = sizeof(s_text_tx_buf) - 1U;
    }

    memcpy(s_text_tx_buf, text, len);
    s_text_tx_buf[len] = '\0';

    (void)USB_Bridge_SendFrame((const uint8_t *)s_text_tx_buf, (uint16_t)len);
}

uint8_t USB_Bridge_SendFrame(const uint8_t *buf, uint16_t len)
{
    if ((buf == NULL) || (len == 0U))
    {
        return 0U;
    }

    if (CDC_Transmit_FS((uint8_t *)buf, len) == USBD_BUSY)
    {
        if (s_tx_busy_count < 0xFFFFU)
        {
            s_tx_busy_count++;
        }
        return 0U;
    }

    return 1U;
}

uint16_t USB_Bridge_GetTxBusyCount(void)
{
    return s_tx_busy_count;
}
