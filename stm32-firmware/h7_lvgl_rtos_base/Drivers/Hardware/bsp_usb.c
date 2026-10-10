#include "bsp_usb.h"
#include "usbd_cdc_if.h"
#include <string.h>

#define CMD_VEL_FRAME_SIZE   12U
#define SCENE_FRAME_SIZE      4U
#define GPS_ROUTE_FRAME_SIZE 23U
#define RX_FRAME_MAX_SIZE    GPS_ROUTE_FRAME_SIZE
/* ⚠️ v2：帧尾追加内部控制状态与耗时字段（见 usb_sensor_telemetry_t）。
   v1 的 151 字节布局原样保留，所以 151 -> 183 只是"后面多接了一段"。
   若以后再加字段，继续往后追、并把版本号再 +1，不要改动已有字段的位置。 */
#define SENSOR_FRAME_VERSION   2U
#define SENSOR_FRAME_SIZE    183U

/* 遥测帧因 CDC 忙被丢弃的累计次数 */
static uint16_t s_sensor_tx_busy_count = 0U;

static uint8_t   rx_buf[RX_FRAME_MAX_SIZE];
static uint8_t   rx_idx = 0;
static uint8_t   rx_expected_size = 0U;
static cmd_vel_t cmd_vel = {0};
static scene_cmd_t scene_cmd = {JETSON_SCENE_NONE, 0U, 0U};
static gps_route_cmd_t gps_route_cmd = {JETSON_GPS_ROUTE_NONE, 0U, 0U, 0U, 0.0, 0.0, 0U, 0U};

static void USB_RxReset(void)
{
    rx_idx = 0U;
    rx_expected_size = 0U;
}

static void USB_RxStart(uint8_t header)
{
    rx_buf[0] = header;
    rx_idx = 1U;
    if (header == 0xAAU) rx_expected_size = CMD_VEL_FRAME_SIZE;
    else if (header == 0xBBU) rx_expected_size = SCENE_FRAME_SIZE;
    else rx_expected_size = GPS_ROUTE_FRAME_SIZE;
}

void USB_Init(void)
{
    USB_RxReset();
    memset(rx_buf, 0, sizeof(rx_buf));
    cmd_vel.mode = 0;
    cmd_vel.vx = 0.0f;
    cmd_vel.vz = 0.0f;
    cmd_vel.last_update_tick = 0U;
    cmd_vel.update_sequence = 0U;
    scene_cmd.scene = JETSON_SCENE_NONE;
    scene_cmd.last_update_tick = 0U;
    scene_cmd.update_sequence = 0U;
    gps_route_cmd.command = JETSON_GPS_ROUTE_NONE;
    gps_route_cmd.update_sequence = 0U;
}

void USB_ProcessRxData(uint8_t *pBuf, uint16_t Size)
{
    if (pBuf == NULL || Size == 0U) return;

    for (uint16_t i = 0; i < Size; i++)
    {
        uint8_t byte = pBuf[i];

        /* Byte 0 selects frame type: AA=cmd_vel, BB=scene, DD=GPS route. */
        if (rx_idx == 0U)
        {
            if (byte == 0xAAU || byte == 0xBBU || byte == 0xDDU) USB_RxStart(byte);
            continue;
        }

        if (rx_idx == 1U)
        {
            if (byte != 0x55U)
            {
                USB_RxReset();
                if (byte == 0xAAU || byte == 0xBBU || byte == 0xDDU) USB_RxStart(byte);
                continue;
            }
        }

        rx_buf[rx_idx++] = byte;

        if (rx_idx == rx_expected_size)
        {
            uint8_t frame_type = rx_buf[0];
            uint8_t checksum = 0U;
            uint8_t j;

            USB_RxReset();

            if (frame_type == 0xAAU)
            {
                /* cmd_vel XOR: Byte2 ~ Byte10. */
                for (j = 2U; j < 11U; j++) checksum ^= rx_buf[j];
                if (checksum != rx_buf[11]) continue;

                cmd_vel.mode = rx_buf[2];
                memcpy(&cmd_vel.vx, &rx_buf[3], 4U);
                memcpy(&cmd_vel.vz, &rx_buf[7], 4U);
                cmd_vel.last_update_tick = HAL_GetTick();
                cmd_vel.update_sequence++;
            }
            else if (frame_type == 0xBBU)
            {
                /* scene_cmd XOR is exactly byte 2. Ignore unknown commands. */
                if (rx_buf[3] != rx_buf[2]) continue;
                if (rx_buf[2] != (uint8_t)JETSON_SCENE_INDOOR &&
                    rx_buf[2] != (uint8_t)JETSON_SCENE_OUTDOOR) continue;

                scene_cmd.scene = (JetsonScene_t)rx_buf[2];
                scene_cmd.last_update_tick = HAL_GetTick();
                scene_cmd.update_sequence++;
            }
            else
            {
                for (j = 2U; j < 22U; j++) checksum ^= rx_buf[j];
                if (checksum != rx_buf[22]) continue;
                if (rx_buf[2] < (uint8_t)JETSON_GPS_ROUTE_BEGIN ||
                    rx_buf[2] > (uint8_t)JETSON_GPS_ROUTE_SPEED) continue;

                gps_route_cmd.command = (JetsonGpsRouteCommand_t)rx_buf[2];
                gps_route_cmd.index = rx_buf[3];
                gps_route_cmd.total = rx_buf[4];
                gps_route_cmd.loop_enable = rx_buf[5] ? 1U : 0U;
                memcpy(&gps_route_cmd.latitude, &rx_buf[6], sizeof(double));
                memcpy(&gps_route_cmd.longitude, &rx_buf[14], sizeof(double));
                gps_route_cmd.last_update_tick = HAL_GetTick();
                gps_route_cmd.update_sequence++;
            }
        }
    }
}

scene_cmd_t USB_GetSceneCmd(void)
{
    return scene_cmd;
}

cmd_vel_t USB_GetCmdVel(void)
{
    return cmd_vel;
}

static void USB_CopyFloat(uint8_t *buf, uint16_t *offset, float value)
{
    memcpy(&buf[*offset], &value, sizeof(float));
    *offset = (uint16_t)(*offset + sizeof(float));
}

gps_route_cmd_t USB_GetGpsRouteCmd(void)
{
    return gps_route_cmd;
}

static void USB_CopyDouble(uint8_t *buf, uint16_t *offset, double value)
{
    memcpy(&buf[*offset], &value, sizeof(double));
    *offset = (uint16_t)(*offset + sizeof(double));
}

/* 16 位小端拷贝。int16 字段也走这里（先转成 uint16_t，位型不变）。 */
static void USB_CopyU16(uint8_t *buf, uint16_t *offset, uint16_t value)
{
    memcpy(&buf[*offset], &value, sizeof(uint16_t));
    *offset = (uint16_t)(*offset + sizeof(uint16_t));
}

uint16_t USB_GetSensorTxBusyCount(void)
{
    return s_sensor_tx_busy_count;
}

void USB_SendSensorTelemetry(const usb_sensor_telemetry_t *telemetry)
{
    uint8_t buf[SENSOR_FRAME_SIZE];
    uint16_t offset = 0U;
    uint8_t checksum = 0U;
    uint16_t i;

    if (telemetry == NULL) return;

    buf[offset++] = 0xCCU;
    buf[offset++] = 0x55U;
    buf[offset++] = SENSOR_FRAME_VERSION;
    buf[offset++] = telemetry->flags;
    buf[offset++] = telemetry->car_mode;
    buf[offset++] = telemetry->satellites;
    buf[offset++] = telemetry->fix_quality;
    buf[offset++] = telemetry->route_total;
    buf[offset++] = telemetry->route_slot;
    buf[offset++] = telemetry->navigation_active;
    buf[offset++] = telemetry->heading_status;
    buf[offset++] = telemetry->loop_enable;
    memcpy(&buf[offset], &telemetry->sequence, sizeof(uint16_t));
    offset = (uint16_t)(offset + sizeof(uint16_t));

    USB_CopyDouble(buf, &offset, telemetry->latitude);
    USB_CopyDouble(buf, &offset, telemetry->longitude);
    USB_CopyDouble(buf, &offset, telemetry->altitude);
    USB_CopyFloat(buf, &offset, telemetry->gnss_heading);
    USB_CopyFloat(buf, &offset, telemetry->gnss_speed);
    USB_CopyFloat(buf, &offset, telemetry->velocity_north);
    USB_CopyFloat(buf, &offset, telemetry->velocity_east);
    USB_CopyFloat(buf, &offset, telemetry->mag_yaw);
    USB_CopyFloat(buf, &offset, telemetry->mag_pitch);
    USB_CopyFloat(buf, &offset, telemetry->mag_roll);
    USB_CopyFloat(buf, &offset, telemetry->imu_roll);
    USB_CopyFloat(buf, &offset, telemetry->imu_pitch);
    USB_CopyFloat(buf, &offset, telemetry->imu_yaw);
    USB_CopyFloat(buf, &offset, telemetry->gyro_x);
    USB_CopyFloat(buf, &offset, telemetry->gyro_y);
    USB_CopyFloat(buf, &offset, telemetry->gyro_z);
    USB_CopyFloat(buf, &offset, telemetry->acc_x);
    USB_CopyFloat(buf, &offset, telemetry->acc_y);
    USB_CopyFloat(buf, &offset, telemetry->acc_z);
    for (i = 0U; i < 4U; i++) USB_CopyFloat(buf, &offset, telemetry->motor_speed[i]);
    USB_CopyDouble(buf, &offset, telemetry->target_latitude);
    USB_CopyDouble(buf, &offset, telemetry->target_longitude);
    USB_CopyDouble(buf, &offset, telemetry->route_latitude);
    USB_CopyDouble(buf, &offset, telemetry->route_longitude);

    /* ---- v2 追加段：内部控制状态 + 控制环耗时 ---- */
    USB_CopyU16(buf, &offset, telemetry->loop_period_ms);
    USB_CopyU16(buf, &offset, telemetry->busy_feedback_ms);
    USB_CopyU16(buf, &offset, telemetry->busy_roadcls_ms);
    USB_CopyU16(buf, &offset, telemetry->busy_control_ms);
    USB_CopyU16(buf, &offset, telemetry->busy_send_ms);
    for (i = 0U; i < 4U; i++)
    {
        USB_CopyU16(buf, &offset, (uint16_t)telemetry->speed_set[i]);
    }
    for (i = 0U; i < 4U; i++)
    {
        USB_CopyU16(buf, &offset, (uint16_t)telemetry->duty[i]);
    }
    USB_CopyFloat(buf, &offset, telemetry->dist_counts);
    USB_CopyU16(buf, &offset, telemetry->tx_busy_count);

    /* 长度自检：与 SENSOR_FRAME_SIZE 不符就直接不发（宁可整帧不出去，
       也不要发出错位的帧让上位机解析出垃圾）。改字段后这里会立刻暴露。 */
    if (offset != (SENSOR_FRAME_SIZE - 1U)) return;
    for (i = 2U; i < offset; i++) checksum ^= buf[i];
    buf[offset] = checksum;

    /* ⚠️ CDC_Transmit_FS 在上一帧还没发完时返回 USBD_BUSY 并【直接丢帧】
       （见 usbd_cdc_if.c 的 TxState 判据）。这里累计丢帧数供上位机判断
       遥测率是不是被 USB 限制住了。 */
    if (CDC_Transmit_FS(buf, SENSOR_FRAME_SIZE) == USBD_BUSY)
    {
        if (s_sensor_tx_busy_count < 0xFFFFU)
        {
            s_sensor_tx_busy_count++;
        }
    }
}
