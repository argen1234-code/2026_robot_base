#include "bsp_bluetooth.h"
#include "usart.h"
#include <string.h>

#define BT_TOKEN_MAX_LEN      16U
#define BT_FRAME_MAX_LEN      64U
#define BT_ONLINE_TIMEOUT_MS  3000U

volatile BT_Debug_t g_bt_debug;

static volatile uint8_t      bt_cmd       = 'S';
static volatile uint8_t      bt_key_state = 0U;
static volatile BT_ModeReq_t bt_mode_req  = BT_MODE_REQ_NONE;
static volatile uint8_t      bt_ack_count = 0U;
static volatile uint8_t      bt_error_count = 0U;
static volatile uint32_t     bt_last_rx_tick = 0U;
static volatile BT_RoadDisplay_t bt_road_display = BT_ROAD_DISPLAY_NOT_STARTED;
static volatile uint8_t      bt_remote_add_waiting = 0U;
static volatile uint8_t      bt_remote_point_pending = 0U;
static volatile uint8_t      bt_powerless_latched = 0U;
static BT_RemotePoint_t      bt_remote_point = {0.0, 0.0};
static char                  bt_frame_buf[BT_FRAME_MAX_LEN];
static uint8_t               bt_frame_len = 0U;
static uint8_t               bt_frame_active = 0U;
static uint8_t               bt_frame_saw_cr = 0U;

static BT_Motion_t bt_debug_motion_from_state(void)
{
    uint8_t cmd = bt_cmd;

    if (bt_key_state == 0U)
    {
        cmd = 'S';
    }

    switch (cmd)
    {
        case 'F': case 'f': return BT_MOTION_FORWARD;
        case 'B': case 'b': return BT_MOTION_BACKWARD;
        case 'L': case 'l': return BT_MOTION_LEFT;
        case 'R': case 'r': return BT_MOTION_RIGHT;
        case 'Q': case 'q': return BT_MOTION_ROTATE_LEFT;
        case 'E': case 'e': return BT_MOTION_ROTATE_RIGHT;
        default:            return BT_MOTION_STOP;
    }
}

static void bt_debug_reset(void)
{
    uint16_t i;

    g_bt_debug.update_sequence = 0U;
    g_bt_debug.rx_event_count = 0U;
    g_bt_debug.rx_byte_count = 0U;
    g_bt_debug.complete_frame_count = 0U;
    g_bt_debug.accepted_token_count = 0U;
    g_bt_debug.ack_queued_count = 0U;
    g_bt_debug.error_queued_count = 0U;
    g_bt_debug.frame_overflow_count = 0U;
    g_bt_debug.format_error_count = 0U;
    g_bt_debug.last_rx_event_tick = 0U;
    g_bt_debug.last_valid_frame_tick = 0U;
    g_bt_debug.last_rx_size = 0U;
    g_bt_debug.last_rx_copied_len = 0U;
    g_bt_debug.frame_active = 0U;
    g_bt_debug.frame_saw_cr = 0U;
    g_bt_debug.frame_len = 0U;
    g_bt_debug.powerless_latched = 0U;
    g_bt_debug.last_payload_len = 0U;
    g_bt_debug.cmd = 'S';
    g_bt_debug.key_state = 0U;
    g_bt_debug.active = 0U;
    g_bt_debug.online = 0U;
    g_bt_debug.motion = BT_MOTION_STOP;
    g_bt_debug.pending_mode_req = BT_MODE_REQ_NONE;
    g_bt_debug.last_mode_req = BT_MODE_REQ_NONE;
    g_bt_debug.ack_pending = 0U;
    g_bt_debug.error_pending = 0U;
    g_bt_debug.last_ack_count = 0U;
    g_bt_debug.remote_add_waiting = 0U;
    g_bt_debug.remote_point_pending = 0U;
    g_bt_debug.remote_point.lat = 0.0;
    g_bt_debug.remote_point.lon = 0.0;

    for (i = 0U; i < BT_DEBUG_RAW_MAX_LEN; i++)
    {
        g_bt_debug.raw_data[i] = 0U;
    }

    for (i = 0U; i < BT_DEBUG_PAYLOAD_MAX_LEN; i++)
    {
        g_bt_debug.last_payload[i] = '\0';
    }
}

static void bt_debug_sync_state(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t active = (bt_key_state != 0U) ? 1U : 0U;

    g_bt_debug.update_sequence++;
    g_bt_debug.frame_active = bt_frame_active;
    g_bt_debug.frame_saw_cr = bt_frame_saw_cr;
    g_bt_debug.frame_len = bt_frame_len;
    g_bt_debug.powerless_latched = bt_powerless_latched;
    g_bt_debug.cmd = bt_cmd;
    g_bt_debug.key_state = bt_key_state;
    g_bt_debug.active = active;
    g_bt_debug.online = (active ||
                         (bt_last_rx_tick != 0U &&
                          (now - bt_last_rx_tick) <= BT_ONLINE_TIMEOUT_MS)) ? 1U : 0U;
    g_bt_debug.motion = bt_debug_motion_from_state();
    g_bt_debug.pending_mode_req = bt_mode_req;
    if (bt_mode_req != BT_MODE_REQ_NONE)
    {
        g_bt_debug.last_mode_req = bt_mode_req;
    }
    g_bt_debug.ack_pending = bt_ack_count;
    g_bt_debug.error_pending = bt_error_count;
    g_bt_debug.remote_add_waiting = bt_remote_add_waiting;
    g_bt_debug.remote_point_pending = bt_remote_point_pending;
    g_bt_debug.remote_point.lat = bt_remote_point.lat;
    g_bt_debug.remote_point.lon = bt_remote_point.lon;
    g_bt_debug.update_sequence++;
}

static void bt_debug_capture_rx(const uint8_t *pBuf, uint16_t Size)
{
    uint16_t i;
    uint16_t copy_len = Size;

    if (copy_len > BT_DEBUG_RAW_MAX_LEN)
    {
        copy_len = BT_DEBUG_RAW_MAX_LEN;
    }

    g_bt_debug.update_sequence++;
    g_bt_debug.rx_event_count++;
    g_bt_debug.rx_byte_count += Size;
    g_bt_debug.last_rx_event_tick = HAL_GetTick();
    g_bt_debug.last_rx_size = Size;
    g_bt_debug.last_rx_copied_len = copy_len;

    for (i = 0U; i < copy_len; i++)
    {
        g_bt_debug.raw_data[i] = pBuf[i];
    }

    g_bt_debug.update_sequence++;
}

static void bt_debug_capture_complete_frame(void)
{
    uint8_t i;

    g_bt_debug.update_sequence++;
    g_bt_debug.complete_frame_count++;
    g_bt_debug.last_valid_frame_tick = bt_last_rx_tick;
    g_bt_debug.last_payload_len = bt_frame_len;

    for (i = 0U; i < bt_frame_len; i++)
    {
        g_bt_debug.last_payload[i] = bt_frame_buf[i];
    }
    g_bt_debug.last_payload[bt_frame_len] = '\0';
    g_bt_debug.update_sequence++;
}

static char bt_to_lower(char c)
{
    if (c >= 'A' && c <= 'Z')
    {
        return (char)(c - 'A' + 'a');
    }
    return c;
}

static uint8_t bt_payload_equals(const char *payload, const char *token)
{
    uint16_t i = 0U;

    if (payload == NULL || token == NULL) return 0U;
    while (payload[i] != '\0' && token[i] != '\0')
    {
        if (bt_to_lower(payload[i]) != token[i]) return 0U;
        i++;
    }
    return (payload[i] == '\0' && token[i] == '\0') ? 1U : 0U;
}

static uint8_t bt_is_token_char(char c)
{
    return ((c >= 'a' && c <= 'z') ||
            (c >= 'A' && c <= 'Z') ||
            (c >= '0' && c <= '9') ||
            c == '_') ? 1U : 0U;
}

static void bt_clear_motion(void)
{
    bt_cmd = 'S';
    bt_key_state = 0U;
}

static void bt_enter_powerless(void)
{
    bt_powerless_latched = 1U;
    bt_clear_motion();
    bt_mode_req = BT_MODE_REQ_NONE;
    bt_remote_add_waiting = 0U;
    bt_remote_point_pending = 0U;
}

static void bt_exit_powerless(void)
{
    bt_powerless_latched = 0U;
    bt_clear_motion();
    bt_mode_req = BT_MODE_REQ_NONE;
    bt_remote_add_waiting = 0U;
    bt_remote_point_pending = 0U;
}

static void bt_queue_ok_response(void)
{
    if (bt_ack_count < 255U)
    {
        bt_ack_count++;
        g_bt_debug.ack_queued_count++;
    }
}

static void bt_queue_ack(void)
{
    g_bt_debug.accepted_token_count++;
    bt_queue_ok_response();
}

static void bt_queue_error_response(void)
{
    if (bt_error_count < 255U)
    {
        bt_error_count++;
        g_bt_debug.error_queued_count++;
    }
}

static uint8_t bt_parse_decimal_component(const char **cursor,
                                          char terminator,
                                          double minimum,
                                          double maximum,
                                          double *value)
{
    const char *p;
    double result = 0.0;
    double fraction_scale = 0.1;
    uint8_t negative = 0U;
    uint8_t integer_digits = 0U;
    uint8_t fraction_digits = 0U;

    if (cursor == NULL || *cursor == NULL || value == NULL) return 0U;

    p = *cursor;
    if (*p == '+' || *p == '-')
    {
        negative = (*p == '-') ? 1U : 0U;
        p++;
    }

    while (*p >= '0' && *p <= '9')
    {
        result = result * 10.0 + (double)(*p - '0');
        integer_digits++;
        p++;
    }

    if (integer_digits == 0U || *p != '.') return 0U;
    p++;

    while (*p >= '0' && *p <= '9')
    {
        result += (double)(*p - '0') * fraction_scale;
        fraction_scale *= 0.1;
        fraction_digits++;
        p++;
    }

    if (fraction_digits < 8U || *p != terminator) return 0U;
    if (negative) result = -result;
    if (result < minimum || result > maximum) return 0U;

    *value = result;
    *cursor = (terminator == '\0') ? p : (p + 1);
    return 1U;
}

static uint8_t bt_parse_remote_point(const char *payload, BT_RemotePoint_t *point)
{
    const char *cursor = payload;

    if (payload == NULL || point == NULL || payload[0] == '\0') return 0U;
    if (!bt_parse_decimal_component(&cursor, ',', -90.0, 90.0, &point->lat)) return 0U;
    if (!bt_parse_decimal_component(&cursor, '\0', -180.0, 180.0, &point->lon)) return 0U;
    return (*cursor == '\0') ? 1U : 0U;
}

static void bt_toggle_motion(uint8_t key_bit, uint8_t cmd)
{
    bt_key_state ^= key_bit;
    bt_cmd = (bt_key_state == 0U) ? 'S' : cmd;
    bt_mode_req = BT_MODE_REQ_INDOOR;
}

static uint8_t bt_is_legacy_cmd_char(char ch)
{
    switch (ch)
    {
        case 'f':
        case 'b':
        case 'l':
        case 'r':
        case 'q':
        case 'e':
        case 's':
        case 'i':
        case 'g':
            return 1U;

        default:
            return 0U;
    }
}

static uint8_t bt_process_legacy_cmd_char(char ch)
{
    switch (ch)
    {
        case 'g':
            bt_mode_req = BT_MODE_REQ_GPS;
            bt_clear_motion();
            return 1U;

        case 'i':
            bt_clear_motion();
            bt_mode_req = BT_MODE_REQ_INDOOR;
            return 1U;

        case 'f':
            bt_toggle_motion(BT_KEY_FORWARD, 'F');
            return 1U;

        case 'b':
            bt_toggle_motion(BT_KEY_BACKWARD, 'B');
            return 1U;

        case 'l':
            bt_toggle_motion(BT_KEY_LEFT, 'L');
            return 1U;

        case 'r':
            bt_toggle_motion(BT_KEY_RIGHT, 'R');
            return 1U;

        case 'q':
            bt_toggle_motion(BT_KEY_ROTATE_LEFT, 'Q');
            return 1U;

        case 'e':
            bt_toggle_motion(BT_KEY_ROTATE_RIGHT, 'E');
            return 1U;

        case 's':
            bt_clear_motion();
            bt_mode_req = BT_MODE_REQ_INDOOR;
            return 1U;

        default:
            break;
    }

    return 0U;
}

static uint8_t bt_process_road_token(const char *token)
{
    if (token == NULL || token[0] == '\0')
    {
        return 0U;
    }

    if (strcmp(token, "road_asphalt") == 0)
    {
        BT_SetRoadDisplay(BT_ROAD_DISPLAY_ASPHALT);
        return 1U;
    }

    if (strcmp(token, "road_indoor") == 0)
    {
        BT_SetRoadDisplay(BT_ROAD_DISPLAY_INDOOR);
        return 1U;
    }

    if (strcmp(token, "road_cement") == 0)
    {
        BT_SetRoadDisplay(BT_ROAD_DISPLAY_OUTDOOR_CEMENT);
        return 1U;
    }

    if (strcmp(token, "road_marble") == 0)
    {
        BT_SetRoadDisplay(BT_ROAD_DISPLAY_OUTDOOR_MARBLE);
        return 1U;
    }

    return 0U;
}

static uint8_t bt_process_token(const char *token)
{
    if (token == NULL || token[0] == '\0')
    {
        return 0U;
    }

    if (bt_process_road_token(token))
    {
        return 1U;
    }

    if (strcmp(token, "g") == 0 || strcmp(token, "gps") == 0)
    {
        bt_mode_req = BT_MODE_REQ_GPS;
        bt_clear_motion();
        return 1U;
    }

    if (strcmp(token, "gps_add") == 0 ||
        strcmp(token, "gps_point") == 0 ||
        strcmp(token, "gps_sample") == 0)
    {
        bt_mode_req = BT_MODE_REQ_GPS_ADD_POINT;
        return 1U;
    }

    if (strcmp(token, "gps_clear") == 0)
    {
        bt_mode_req = BT_MODE_REQ_GPS_CLEAR_POINTS;
        return 1U;
    }

    if (strcmp(token, "i") == 0 ||
        strcmp(token, "indoor") == 0 ||
        strcmp(token, "bt") == 0 ||
        strcmp(token, "bluetooth") == 0)
    {
        bt_clear_motion();
        bt_mode_req = BT_MODE_REQ_INDOOR;
        return 1U;
    }

    if (strcmp(token, "f") == 0 || strcmp(token, "forward") == 0)
    {
        bt_toggle_motion(BT_KEY_FORWARD, 'F');
        return 1U;
    }

    if (strcmp(token, "b") == 0 ||
        strcmp(token, "back") == 0 ||
        strcmp(token, "backward") == 0)
    {
        bt_toggle_motion(BT_KEY_BACKWARD, 'B');
        return 1U;
    }

    if (strcmp(token, "l") == 0 || strcmp(token, "left") == 0)
    {
        bt_toggle_motion(BT_KEY_LEFT, 'L');
        return 1U;
    }

    if (strcmp(token, "r") == 0 || strcmp(token, "right") == 0)
    {
        bt_toggle_motion(BT_KEY_RIGHT, 'R');
        return 1U;
    }

    if (strcmp(token, "s") == 0 || strcmp(token, "stop") == 0)
    {
        bt_clear_motion();
        bt_mode_req = BT_MODE_REQ_INDOOR;
        return 1U;
    }

    if (strcmp(token, "q") == 0 ||
        strcmp(token, "ccw") == 0 ||
        strcmp(token, "turn_left") == 0 ||
        strcmp(token, "rotate_left") == 0)
    {
        bt_toggle_motion(BT_KEY_ROTATE_LEFT, 'Q');
        return 1U;
    }

    if (strcmp(token, "e") == 0 ||
        strcmp(token, "cw") == 0 ||
        strcmp(token, "turn_right") == 0 ||
        strcmp(token, "rotate_right") == 0)
    {
        bt_toggle_motion(BT_KEY_ROTATE_RIGHT, 'E');
        return 1U;
    }

    {
        uint8_t i = 0U;
        uint8_t handled = 0U;

        while (token[i] != '\0')
        {
            if (!bt_is_legacy_cmd_char(token[i]))
            {
                return 0U;
            }
            i++;
        }

        for (i = 0U; token[i] != '\0'; i++)
        {
            handled |= bt_process_legacy_cmd_char(token[i]);
        }

        return handled;
    }
}

static void bt_process_payload(const char *payload)
{
    char token[BT_TOKEN_MAX_LEN];
    uint8_t token_len = 0U;
    uint16_t i = 0U;

    if (payload == NULL)
    {
        return;
    }

    /* Highest-priority one-symbol coast command, valid in every parser state. */
    if (strcmp(payload, "!") == 0)
    {
        bt_enter_powerless();
        bt_queue_ack();
        return;
    }

    /* Recovery never resumes an old command; a new mode/motion command is required. */
    if (strcmp(payload, "~") == 0 ||
        bt_payload_equals(payload, "resume") ||
        bt_payload_equals(payload, "power_on"))
    {
        bt_exit_powerless();
        bt_queue_ack();
        return;
    }

    if (bt_powerless_latched)
    {
        bt_queue_error_response();
        return;
    }

    if (bt_remote_add_waiting)
    {
        BT_RemotePoint_t point;

        /* The coordinate state is one-shot, regardless of parse success. */
        bt_remote_add_waiting = 0U;
        if (!bt_remote_point_pending && bt_parse_remote_point(payload, &point))
        {
            bt_remote_point = point;
            bt_remote_point_pending = 1U;
        }
        else
        {
            bt_queue_error_response();
        }
        return;
    }

    if (bt_payload_equals(payload, "remote_add"))
    {
        bt_remote_add_waiting = 1U;
        bt_queue_ack();
        return;
    }

    while (payload[i] != '\0')
    {
        char ch = payload[i++];

        if (bt_is_token_char(ch))
        {
            if (token_len < (BT_TOKEN_MAX_LEN - 1U))
            {
                token[token_len++] = bt_to_lower(ch);
            }
        }
        else
        {
            token[token_len] = '\0';
            if (bt_process_token(token))
            {
                bt_queue_ack();
            }
            token_len = 0U;
        }
    }

    token[token_len] = '\0';
    if (bt_process_token(token))
    {
        bt_queue_ack();
    }
}

static void bt_frame_reset(void)
{
    bt_frame_len = 0U;
    bt_frame_active = 0U;
    bt_frame_saw_cr = 0U;
}

void BT_Init(void)
{
    bt_cmd       = 'S';
    bt_key_state = 0U;
    bt_mode_req  = BT_MODE_REQ_NONE;
    bt_ack_count = 0U;
    bt_error_count = 0U;
    bt_last_rx_tick = 0U;
    bt_road_display = BT_ROAD_DISPLAY_NOT_STARTED;
    bt_remote_add_waiting = 0U;
    bt_remote_point_pending = 0U;
    bt_powerless_latched = 0U;
    bt_remote_point.lat = 0.0;
    bt_remote_point.lon = 0.0;
    bt_frame_reset();
    bt_debug_reset();
    bt_debug_sync_state();
}

/* Called from UART ISR / DMA callback to feed received data */
void BT_ProcessRxData(uint8_t *pBuf, uint16_t Size)
{
    if (pBuf == NULL || Size == 0U)
    {
        return;
    }

    bt_debug_capture_rx(pBuf, Size);

    for (uint16_t i = 0; i < Size; i++)
    {
        char ch = (char)pBuf[i];

        if (!bt_frame_active)
        {
            if (ch == '@')
            {
                bt_frame_active = 1U;
                bt_frame_len = 0U;
                bt_frame_saw_cr = 0U;
            }
            continue;
        }

        if (ch == '@')
        {
            bt_frame_active = 1U;
            bt_frame_len = 0U;
            bt_frame_saw_cr = 0U;
            continue;
        }

        if (bt_frame_saw_cr)
        {
            if (ch == '\n')
            {
                bt_frame_buf[bt_frame_len] = '\0';
                bt_last_rx_tick = HAL_GetTick();
                bt_debug_capture_complete_frame();
                bt_process_payload(bt_frame_buf);
                bt_frame_reset();
            }
            else if (ch == '@')
            {
                bt_frame_active = 1U;
                bt_frame_len = 0U;
                bt_frame_saw_cr = 0U;
            }
            else
            {
                g_bt_debug.format_error_count++;
                if (bt_remote_add_waiting)
                {
                    bt_remote_add_waiting = 0U;
                    bt_queue_error_response();
                }
                bt_frame_reset();
            }
            continue;
        }

        if (ch == '\r')
        {
            bt_frame_saw_cr = 1U;
            continue;
        }

        if (bt_frame_len < (BT_FRAME_MAX_LEN - 1U))
        {
            bt_frame_buf[bt_frame_len++] = ch;
        }
        else
        {
            g_bt_debug.frame_overflow_count++;
            if (bt_remote_add_waiting)
            {
                bt_remote_add_waiting = 0U;
                bt_queue_error_response();
            }
            bt_frame_reset();
        }
    }

    bt_debug_sync_state();
}

/*
 * The phone application may end a road_* command with a UART idle gap instead
 * of transmitting CR/LF. Accept only the four display commands in this path;
 * motion and mode commands keep the original @payload\r\n protocol unchanged.
 */
void BT_ProcessRxIdle(void)
{
    char token[BT_TOKEN_MAX_LEN];
    uint8_t i;

    if (!bt_frame_active || bt_frame_saw_cr || bt_frame_len == 0U)
    {
        return;
    }

    if (bt_remote_add_waiting)
    {
        /* Remote coordinates always require the complete @lat,lon\r\n frame. */
        bt_remote_add_waiting = 0U;
        bt_queue_error_response();
        bt_frame_reset();
        bt_debug_sync_state();
        return;
    }

    if (bt_frame_len >= BT_TOKEN_MAX_LEN)
    {
        return;
    }

    for (i = 0U; i < bt_frame_len; i++)
    {
        if (!bt_is_token_char(bt_frame_buf[i]))
        {
            return;
        }
        token[i] = bt_to_lower(bt_frame_buf[i]);
    }
    token[bt_frame_len] = '\0';

    if (!bt_process_road_token(token))
    {
        return;
    }

    bt_frame_buf[bt_frame_len] = '\0';
    bt_last_rx_tick = HAL_GetTick();
    bt_debug_capture_complete_frame();
    bt_queue_ack();
    bt_frame_reset();
    bt_debug_sync_state();
}

/* Returns 1 if BT remote is actively sending motion commands */
uint8_t BT_IsActive(void)
{
    uint8_t active = (bt_key_state != 0U) ? 1U : 0U;

    g_bt_debug.active = active;
    return active;
}

uint8_t BT_IsOnline(void)
{
    uint32_t tick = bt_last_rx_tick;
    uint8_t online;

    if (BT_IsActive())
    {
        online = 1U;
    }
    else
    {
        online = (tick != 0U && (HAL_GetTick() - tick) <= BT_ONLINE_TIMEOUT_MS) ? 1U : 0U;
    }

    g_bt_debug.online = online;
    return online;
}

uint8_t BT_GetKeyState(void)
{
    g_bt_debug.key_state = bt_key_state;
    return bt_key_state;
}

uint8_t BT_IsPowerless(void)
{
    g_bt_debug.powerless_latched = bt_powerless_latched;
    return bt_powerless_latched;
}

BT_RoadDisplay_t BT_GetRoadDisplay(void)
{
    return bt_road_display;
}

void BT_SetRoadDisplay(BT_RoadDisplay_t road)
{
    if (road < BT_ROAD_DISPLAY_COUNT)
    {
        bt_road_display = road;
    }
}

/* Returns current motion command, auto-fallback to STOP on timeout */
BT_Motion_t BT_GetMotion(void)
{
    uint8_t cmd = bt_cmd;
    BT_Motion_t motion;

    if (!BT_IsActive())
    {
        cmd = 'S';
    }

    switch (cmd)
    {
        case 'F': case 'f': motion = BT_MOTION_FORWARD; break;
        case 'B': case 'b': motion = BT_MOTION_BACKWARD; break;
        case 'L': case 'l': motion = BT_MOTION_LEFT; break;
        case 'R': case 'r': motion = BT_MOTION_RIGHT; break;
        case 'Q': case 'q': motion = BT_MOTION_ROTATE_LEFT; break;
        case 'E': case 'e': motion = BT_MOTION_ROTATE_RIGHT; break;
        default:            motion = BT_MOTION_STOP; break;
    }

    g_bt_debug.motion = motion;
    return motion;
}

/* Returns and clears the pending mode request from BT */
BT_ModeReq_t BT_GetAndClearModeReq(void)
{
    BT_ModeReq_t req = bt_mode_req;
    bt_mode_req = BT_MODE_REQ_NONE;
    if (req != BT_MODE_REQ_NONE)
    {
        g_bt_debug.last_mode_req = req;
    }
    bt_debug_sync_state();
    return req;
}

uint8_t BT_GetAndClearRemotePoint(BT_RemotePoint_t *point)
{
    uint8_t pending;
    uint32_t primask;

    if (point == NULL) return 0U;

    primask = __get_PRIMASK();
    __disable_irq();
    pending = bt_remote_point_pending;
    if (pending)
    {
        *point = bt_remote_point;
        bt_remote_point_pending = 0U;
    }
    if (primask == 0U)
    {
        __enable_irq();
    }

    bt_debug_sync_state();
    return pending;
}

void BT_ReportRemotePointResult(uint8_t success)
{
    if (success)
    {
        bt_queue_ok_response();
    }
    else
    {
        bt_queue_error_response();
    }
    bt_debug_sync_state();
}

uint8_t BT_GetAndClearAckCount(void)
{
    uint8_t count;

    __disable_irq();
    count = bt_ack_count;
    bt_ack_count = 0U;
    __enable_irq();

    g_bt_debug.last_ack_count = count;
    bt_debug_sync_state();
    return count;
}

void BT_ServicePendingAck(void)
{
    static const uint8_t ack_msg[] = {'o', 'k', '\r', '\n'};
    static const uint8_t error_msg[] = {'e', 'r', 'r', 'o', 'r', '\r', '\n'};
    uint8_t sent_count = 0U;
    uint32_t primask;

    while (bt_ack_count > 0U)
    {
        /* Keep the ACK pending when USART1 is temporarily occupied by startup printf output. */
        if (HAL_UART_Transmit(&huart1, (uint8_t *)ack_msg, sizeof(ack_msg), 10U) != HAL_OK)
        {
            break;
        }

        primask = __get_PRIMASK();
        __disable_irq();
        if (bt_ack_count > 0U)
        {
            bt_ack_count--;
        }
        if (primask == 0U)
        {
            __enable_irq();
        }
        sent_count++;
    }

    while (bt_error_count > 0U)
    {
        if (HAL_UART_Transmit(&huart1, (uint8_t *)error_msg, sizeof(error_msg), 10U) != HAL_OK)
        {
            break;
        }

        primask = __get_PRIMASK();
        __disable_irq();
        if (bt_error_count > 0U)
        {
            bt_error_count--;
        }
        if (primask == 0U)
        {
            __enable_irq();
        }
    }

    if (sent_count > 0U)
    {
        g_bt_debug.last_ack_count = sent_count;
    }
    bt_debug_sync_state();
}
