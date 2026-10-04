#ifndef BSP_BLUETOOTH_H
#define BSP_BLUETOOTH_H

#include "main.h"

#define BT_KEY_FORWARD       0x01U
#define BT_KEY_BACKWARD      0x02U
#define BT_KEY_LEFT          0x04U
#define BT_KEY_RIGHT         0x08U
#define BT_KEY_ROTATE_LEFT   0x10U
#define BT_KEY_ROTATE_RIGHT  0x20U

/* Motion direction from Bluetooth remote control */
typedef enum {
    BT_MOTION_STOP = 0,
    BT_MOTION_FORWARD,
    BT_MOTION_BACKWARD,
    BT_MOTION_LEFT,
    BT_MOTION_RIGHT,
    BT_MOTION_ROTATE_LEFT,
    BT_MOTION_ROTATE_RIGHT
} BT_Motion_t;

/* Mode/action request received over Bluetooth */
typedef enum {
    BT_MODE_REQ_NONE = 0,
    BT_MODE_REQ_GPS,
    BT_MODE_REQ_INDOOR,
    BT_MODE_REQ_GPS_ADD_POINT,
    BT_MODE_REQ_GPS_CLEAR_POINTS
} BT_ModeReq_t;

typedef struct {
    double lat;
    double lon;
} BT_RemotePoint_t;

/* Shared road display selected by the classifier, Bluetooth, or the LVGL button. */
typedef uint8_t BT_RoadDisplay_t;
#define BT_ROAD_DISPLAY_NOT_STARTED      ((BT_RoadDisplay_t)0U)
#define BT_ROAD_DISPLAY_INDOORS          ((BT_RoadDisplay_t)1U)
#define BT_ROAD_DISPLAY_CEMENT           ((BT_RoadDisplay_t)2U)
#define BT_ROAD_DISPLAY_ASPHALT          ((BT_RoadDisplay_t)3U)
#define BT_ROAD_DISPLAY_OUTDOORS         ((BT_RoadDisplay_t)4U)
#define BT_ROAD_DISPLAY_COUNT            ((BT_RoadDisplay_t)5U)

/* Compatibility aliases for the existing Bluetooth text commands. */
#define BT_ROAD_DISPLAY_MARBLE           BT_ROAD_DISPLAY_INDOORS
#define BT_ROAD_DISPLAY_INDOOR           BT_ROAD_DISPLAY_INDOORS
#define BT_ROAD_DISPLAY_OUTDOOR_MARBLE   BT_ROAD_DISPLAY_INDOORS
#define BT_ROAD_DISPLAY_OUTDOOR_CEMENT   BT_ROAD_DISPLAY_CEMENT

/*
 * Keil Watch debug mirror for the Bluetooth receive path.
 *
 * g_bt_debug is written only for observation; the vehicle control path never
 * reads it. raw_data contains the most recent UART RX event exactly as passed
 * to BT_ProcessRxData(), while last_payload contains the most recent complete
 * frame payload (without '@' and CR/LF).
 */
#define BT_DEBUG_RAW_MAX_LEN      512U
#define BT_DEBUG_PAYLOAD_MAX_LEN   64U

typedef struct
{
    uint32_t update_sequence;       /* Changes around each mirror update. */

    uint32_t rx_event_count;        /* UART RX-to-idle callback count. */
    uint32_t rx_byte_count;         /* Total bytes observed by BT parser. */
    uint32_t complete_frame_count;  /* CRLF frames plus road-command idle fallback. */
    uint32_t accepted_token_count;  /* Frames/tokens that queued an ACK. */
    uint32_t ack_queued_count;      /* Number of ACKs queued successfully. */
    uint32_t error_queued_count;    /* Number of error responses queued. */
    uint32_t frame_overflow_count;  /* Payloads exceeding parser capacity. */
    uint32_t format_error_count;    /* CR not followed by LF. */

    uint32_t last_rx_event_tick;    /* HAL tick of the latest DMA event. */
    uint32_t last_valid_frame_tick; /* HAL tick of the latest complete frame. */
    uint16_t last_rx_size;          /* Byte count reported by the UART callback. */
    uint16_t last_rx_copied_len;    /* Bytes copied into raw_data. */
    uint8_t  raw_data[BT_DEBUG_RAW_MAX_LEN];

    uint8_t  frame_active;          /* Parser has seen '@'. */
    uint8_t  frame_saw_cr;          /* Parser is waiting for LF. */
    uint8_t  frame_len;             /* Current partial payload length. */
    uint8_t  powerless_latched;    /* @! enters; @~, @resume or @power_on clears it. */
    char     last_payload[BT_DEBUG_PAYLOAD_MAX_LEN];
    uint8_t  last_payload_len;

    uint8_t      cmd;               /* Current command character, e.g. 'F'. */
    uint8_t      key_state;         /* BT_KEY_* bit mask. */
    uint8_t      active;            /* Same meaning as BT_IsActive(). */
    uint8_t      online;            /* Same meaning as the latest BT_IsOnline(). */
    BT_Motion_t  motion;            /* Decoded motion. */
    BT_ModeReq_t pending_mode_req;  /* Request not yet consumed by chassis task. */
    BT_ModeReq_t last_mode_req;     /* Last non-NONE request, retained for Watch. */
    uint8_t      ack_pending;       /* ACK count awaiting chassis transmission. */
    uint8_t      error_pending;     /* Error count awaiting chassis transmission. */
    uint8_t      last_ack_count;    /* ACK count last consumed by chassis task. */
    uint8_t      remote_add_waiting;/* Next complete frame must be lat,lon. */
    uint8_t      remote_point_pending;
    BT_RemotePoint_t remote_point;
} BT_Debug_t;

/* Add this single symbol to Keil Watch and expand it. Read-only for users. */
extern volatile BT_Debug_t g_bt_debug;

void           BT_Init(void);
void           BT_ProcessRxData(uint8_t *pBuf, uint16_t Size);
void           BT_ProcessRxIdle(void);
BT_Motion_t    BT_GetMotion(void);
uint8_t        BT_IsActive(void);
uint8_t        BT_IsOnline(void);
uint8_t        BT_GetKeyState(void);
uint8_t        BT_IsPowerless(void);
BT_ModeReq_t   BT_GetAndClearModeReq(void);
uint8_t        BT_GetAndClearRemotePoint(BT_RemotePoint_t *point);
void           BT_ReportRemotePointResult(uint8_t success);
uint8_t        BT_GetAndClearAckCount(void);
void           BT_ServicePendingAck(void);
BT_RoadDisplay_t BT_GetRoadDisplay(void);
void           BT_SetRoadDisplay(BT_RoadDisplay_t road);

#endif
