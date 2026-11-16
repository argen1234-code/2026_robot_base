/*
 * USB 上行遥测：CC55 v2（183 字节），布局与 h7_lvgl_rtos_base 的
 * Drivers/Hardware/bsp_usb.c: USB_SendSensorTelemetry() 完全一致，
 * 上位机解析器是 robot_base_bringup/scripts/chassis_telemetry.py。
 *
 * F407 板上没有 IMU/GPS/磁力计，所以这些字段全填 0，flags 也不置对应有效位；
 * 只填四轮转速/目标转速、模式、故障与控制周期。
 */
#include "app_telemetry.h"
#include "app_chassis_board.h"
#include "app_usb_bridge.h"
#include "main.h"

#include <string.h>

#define TELEMETRY_RATE_MS        50U    /* 20 Hz */
#define SENSOR_FRAME_VERSION     2U
#define SENSOR_FRAME_SIZE        183U

/* 轮速计数刻度：沿用旧工程 ±45 rad/s ↔ ±120 counts（每 10 ms 周期） */
#define COUNTS_PER_RAD_S         (120.0f / 45.0f)

/* CC55 v2 字段偏移（与 chassis_telemetry.py 的 OFF / OFF_V2 对应） */
#define OFF_VERSION              2U
#define OFF_FLAGS                3U
#define OFF_CAR_MODE             4U
#define OFF_NAV_ACTIVE           9U
#define OFF_SEQUENCE             12U
#define OFF_MOTOR_SPEED          102U
#define OFF_LOOP_PERIOD_MS       150U
#define OFF_BUSY_FEEDBACK_MS     152U
#define OFF_BUSY_ROADCLS_MS      154U
#define OFF_BUSY_CONTROL_MS      156U
#define OFF_BUSY_SEND_MS         158U
#define OFF_SPEED_SET            160U
#define OFF_DUTY                 168U
#define OFF_DIST_COUNTS          176U
#define OFF_TX_BUSY_COUNT        180U

static uint8_t  s_frame[SENSOR_FRAME_SIZE];
static uint32_t s_last_tick = 0U;
static uint32_t s_drop_count = 0U;
static uint16_t s_sequence = 0U;

static void put_u16(uint8_t *buf, uint16_t off, uint16_t value)
{
    memcpy(&buf[off], &value, sizeof(value));
}

static void put_f32(uint8_t *buf, uint16_t off, float value)
{
    memcpy(&buf[off], &value, sizeof(value));
}

static void put_f64(uint8_t *buf, uint16_t off, double value)
{
    memcpy(&buf[off], &value, sizeof(value));
}

void Telemetry_Init(void)
{
    memset(s_frame, 0, sizeof(s_frame));
    s_last_tick = 0U;
    s_drop_count = 0U;
    s_sequence = 0U;
}

void Telemetry_Update(chassis_move_t *chassis)
{
    uint32_t now;
    uint8_t checksum = 0U;
    uint16_t i;
    uint8_t car_mode;

    if (chassis == NULL)
    {
        return;
    }

    now = HAL_GetTick();
    if ((now - s_last_tick) < TELEMETRY_RATE_MS)
    {
        return;
    }
    s_last_tick = now;

    memset(s_frame, 0, sizeof(s_frame));

    s_frame[0] = 0xCCU;
    s_frame[1] = 0x55U;
    s_frame[OFF_VERSION] = SENSOR_FRAME_VERSION;
    s_frame[OFF_FLAGS] = 0U;    /* 无 IMU/GPS/磁力计：有效位全 0 */

    /* car_mode 用 JetsonMode_t：2=REMOTE / 3=INDOOR，其余状态报 0(IDLE) */
    car_mode = 0U;
    if (chassis->mode == CAR_MODE_INDOOR)
    {
        car_mode = (chassis->jetson_mode == 2U) ? 2U : 3U;
    }
    s_frame[OFF_CAR_MODE] = car_mode;
    s_frame[OFF_NAV_ACTIVE] = (chassis->mode == CAR_MODE_INDOOR) ? 1U : 0U;

    put_u16(s_frame, OFF_SEQUENCE, s_sequence++);

    for (i = 0U; i < 4U; i++)
    {
        float speed_counts =
            (float)chassis->motor[i].speed * COUNTS_PER_RAD_S;
        int16_t set_counts =
            (int16_t)(chassis->motor[i].speed_set * COUNTS_PER_RAD_S);

        put_f32(s_frame, (uint16_t)(OFF_MOTOR_SPEED + 4U * i),
                speed_counts);
        put_u16(s_frame, (uint16_t)(OFF_SPEED_SET + 2U * i),
                (uint16_t)set_counts);
        put_u16(s_frame, (uint16_t)(OFF_DUTY + 2U * i), 0U);
    }

    put_u16(s_frame, OFF_LOOP_PERIOD_MS, chassis->loop_dt_ms);
    put_u16(s_frame, OFF_BUSY_FEEDBACK_MS, 0U);
    put_u16(s_frame, OFF_BUSY_ROADCLS_MS, 0U);
    put_u16(s_frame, OFF_BUSY_CONTROL_MS, 0U);
    put_u16(s_frame, OFF_BUSY_SEND_MS, 0U);
    put_f32(s_frame, OFF_DIST_COUNTS, 0.0f);
    put_u16(s_frame, OFF_TX_BUSY_COUNT, USB_Bridge_GetTxBusyCount());

    for (i = 2U; i < (SENSOR_FRAME_SIZE - 1U); i++)
    {
        checksum ^= s_frame[i];
    }
    s_frame[SENSOR_FRAME_SIZE - 1U] = checksum;

    if (USB_Bridge_SendFrame(s_frame, SENSOR_FRAME_SIZE) == 0U)
    {
        s_drop_count++;
    }
}

uint32_t Telemetry_GetDropCount(void)
{
    return s_drop_count;
}
